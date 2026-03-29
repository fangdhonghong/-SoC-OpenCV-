#include "v4l2_capture.h"

// ============ 全局变量定义 ============ 

std::atomic<bool> g_shutdown_requested(false);  // 当值为0时，生产者线程退出监听

// 全局帧队列
ThreadSafeQueue<Frame> g_frame_queue(4);

// ============ 信号处理 ============
void sigint_handler(int sig) {
    g_shutdown_requested.store(true);
    g_frame_queue.stop();
}

// 自动检测可用的 ISP 设备节点 => 遍历筛选
const char* find_available_isp_device() {
    char device_path[64];
    static char found_path[64];

    for (int i = 0; i < 32; i++) {
        snprintf(device_path, sizeof(device_path), "/dev/video%d", i);

        int fd = open(device_path, O_RDWR);  // 打开一个
        if (fd < 0) {
            continue;
        }

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {  // 查询设备能力
            close(fd);
            continue;
        }

        if (strstr((const char*)cap.driver, "rkisp") == NULL) {  // 该设备不支持ISP驱动
            close(fd);
            continue;
        }

        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {  // 申请 buffer
            close(fd);
            continue;
        }

        struct v4l2_format check_fmt;
        memset(&check_fmt, 0, sizeof(check_fmt));
        check_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        check_fmt.fmt.pix_mp.num_planes = 1;

        if (ioctl(fd, VIDIOC_G_FMT, &check_fmt) < 0) {  // 查询图像格式
            close(fd);
            continue;
        }

        if (check_fmt.fmt.pix_mp.width != 576 || check_fmt.fmt.pix_mp.height != 324) {  // 目标是 576 * 324的
            printf("Skipping %s: resolution %dx%d (need 576x324)\n",
                   device_path,
                   check_fmt.fmt.pix_mp.width,
                   check_fmt.fmt.pix_mp.height);
            close(fd);
            continue;
        }

        close(fd);
        snprintf(found_path, sizeof(found_path), "/dev/video%d", i);
        printf("Found available ISP device: %s (driver: %s)\n", found_path, cap.driver);
        return found_path;
    }

    return NULL;
}

int main()
{
    const char* device = find_available_isp_device();  // 筛选合适的设备节点
    if (device == NULL) {
        fprintf(stderr, "No available ISP device found\n");
        return -1;
    }

    int fd = open(device, O_RDWR);  // 打开经过筛选的设备节点
    if (fd < 0) {
        perror("open");
        return -1;
    }
    printf("camera opened: %s\n", device);

    // 查询设备能力
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        close(fd);
        return -1;
    }

    printf("driver: %s\n", cap.driver);
    printf("card: %s\n", cap.card);

    // 查询图像格式
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.num_planes = 1;

    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("VIDIOC_G_FMT");
        close(fd);
        return -1;
    }

    printf("width: %d\n", fmt.fmt.pix_mp.width);
    printf("height: %d\n", fmt.fmt.pix_mp.height);
    printf("pixelformat: 0x%x\n", fmt.fmt.pix_mp.pixelformat);

    // 根据图像格式选择对应opencv的解码
    int yuv2bgr_code;
    switch (fmt.fmt.pix_mp.pixelformat) {
        case V4L2_PIX_FMT_NV12:  yuv2bgr_code = cv::COLOR_YUV2BGR_NV12; break;
        case V4L2_PIX_FMT_NV21:  yuv2bgr_code = cv::COLOR_YUV2BGR_NV21; break;
        case V4L2_PIX_FMT_YUYV:  yuv2bgr_code = cv::COLOR_YUV2BGR_YUYV; break;
        case V4L2_PIX_FMT_UYVY:  yuv2bgr_code = cv::COLOR_YUV2BGR_UYVY; break;
        case V4L2_PIX_FMT_YVYU:  yuv2bgr_code = cv::COLOR_YUV2BGR_YVYU; break;
        default:
            fprintf(stderr, "Unsupported pixel format: 0x%x\n", fmt.fmt.pix_mp.pixelformat);
            close(fd);
            return -1;
    }
    printf("YUV->BGR conversion code: %d\n", yuv2bgr_code);

    // 申请 buffer
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        close(fd);
        return -1;
    }

    struct buffer buffers[4];

    // 查询 buffer 信息和实现 mmap 映射
    for (int i = 0; i < 4; i++) {

        struct v4l2_buffer buf;
        struct v4l2_plane planes[1];

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = 1;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            close(fd);
            return -1;
        }

        buffers[i].length = buf.m.planes[0].length;

        buffers[i].start = mmap(
            NULL,
            buf.m.planes[0].length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            fd,
            buf.m.planes[0].m.mem_offset
        );

        // 其中一个查询失败，就把之前申请的 mmap 映射撤回
        if (buffers[i].start == MAP_FAILED) {
            perror("mmap");
            for (int j = 0; j < i; j++) {
                munmap(buffers[j].start, buffers[j].length);
            }
            close(fd);
            return -1;
        }

        printf("buffer %d mapped\n", i);
    }

    // 把 buffer 丢给驱动
    for (int i = 0; i < 4; i++) {

        struct v4l2_buffer buf;
        struct v4l2_plane planes[1];

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = 1;

        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            close(fd);
            return -1;
        }
    }

    // 开始采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        close(fd);
        return -1;
    }

    printf("streaming...\n");

    // 信号处理
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, NULL) < 0) {
        perror("sigaction");
        close(fd);
        return -1;
    }

    // 书写上下文线程：生产者
    int width = fmt.fmt.pix_mp.width;
    int height = fmt.fmt.pix_mp.height;
    size_t frame_size = buffers[0].length;

    struct capture_context cap_ctx = {
        .fd = fd,
        .buffers = buffers,
        .width = width,
        .height = height,
        .pixelformat = fmt.fmt.pix_mp.pixelformat,
        .frame_size = frame_size
    };

    // 书写上下文线程：消费者
    struct process_context proc_ctx = {
        .yuv2bgr_code = yuv2bgr_code,
        .decision = {
            .identified = false,
            .consecutive_misses = 0,
            .max_misses = 5,
            .tracker = {
                .cx = -1,
                .cy = -1,
                .prev_cx = -1,
                .prev_cy = -1,
                .filtered_cx = -1,
                .filtered_cy = -1,
                .filter_initialized = false,
                .alpha = 0.8f,
                .vx = 0.0f,
                .vy = 0.0f,
                .max_delta_distance = 200.0f
            }
        }
    };

    // 创建线程
    std::thread capture_thread(capture_thread_func, &cap_ctx);
    std::thread process_thread(process_thread_func, &proc_ctx);

    // 关闭线程和全局帧队列
    capture_thread.join();

    g_frame_queue.stop();

    process_thread.join();

    // 关闭视频流
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("VIDIOC_STREAMOFF");
        close(fd);
        return -1;
    }

    // 撤销 mmap 映射
    for (int i = 0; i < 4; i++) {
        if (munmap(buffers[i].start, buffers[i].length) < 0) {
            perror("munmap");
        }
    }

    // 关闭节点
    close(fd);
    return 0;
}