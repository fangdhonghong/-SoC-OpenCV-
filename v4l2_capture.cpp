#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <signal.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

// volatile sig_atomic_t 是专门为信号设计的类型，确保在中断时修改它是安全的
volatile sig_atomic_t running = 1;

// 线程间通信的关闭标志
std::atomic<bool> g_shutdown_requested(false);

// Frame 结构体：携带一帧的所有信息
struct Frame {
    std::vector<uint8_t> data;  // buffer的数据
    size_t bytesused;           // 数据量
    int width, height;          // 照片的大小
    uint32_t pixelformat;       // 照片的格式
    int frame_id;               // 第几帧
};

// 线程安全的队列模板类
template<typename T>
class ThreadSafeQueue {
    std::queue<T> queue_;             // 队列
    std::mutex mutex_;                // 互斥锁
    std::condition_variable cond_;    // 条件变量
    size_t max_size_;                 // 队列大小
    bool stop_signaled_;              // 停止信号

public:
    explicit ThreadSafeQueue(size_t max_size) : max_size_(max_size), stop_signaled_(false) {}

    // 禁止拷贝
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    // 入队：队列满时丢弃最老帧，不阻塞生产者（保证实时流）
    bool enqueue(T&& frame) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_signaled_) {
            return false;
        }
        // 队列满则丢弃队首最老的帧，保持实时性
        if (queue_.size() >= max_size_) {
            queue_.pop();
        }
        queue_.push(std::move(frame));
        cond_.notify_one();
        return true;
    }

    // 出队：队列空时阻塞等待
    bool dequeue(T& frame) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_signaled_ && queue_.empty()) {
            return false;
        }
        cond_.wait(lock, [this] { return !queue_.empty() || stop_signaled_; });
        if (stop_signaled_ && queue_.empty()) {
            return false;
        }
        frame = std::move(queue_.front());
        queue_.pop();
        cond_.notify_one();
        return true;
    }

    // 通知所有等待者停止
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_signaled_ = true;
        }
        cond_.notify_all();
    }

    bool is_stopped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_signaled_;
    }
};

// 全局帧队列，容量为4（与V4L2 buffer数量一致）
ThreadSafeQueue<Frame> g_frame_queue(4);

// 收到信号时把开关关掉
void sigint_handler(int sig) {
    printf("\n[Signal] Received Ctrl+C, preparing to exit gracefully...\n");
    running = 0;
    g_shutdown_requested.store(true);
    g_frame_queue.stop();
}

struct buffer {
    void *start;
    size_t length;
};

// 线程上下文结构体
struct capture_context {
    int fd;
    struct buffer* buffers;
    int width;
    int height;
    uint32_t pixelformat;
    size_t frame_size;
};

struct process_context {
    int yuv2bgr_code;
};

// ============ 帧验证辅助函数 ============

// 根据像素格式计算期望的帧大小（bytesused）
size_t calculate_expected_size(int width, int height, uint32_t pixelformat) {
    switch (pixelformat) {
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV21:
            // Y plane: w * h, UV plane: w * h / 2
            return width * height * 3 / 2;
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_YVYU:
            // YUYV 等格式：2 bytes per pixel
            return width * height * 2;
        default:
            return 0;
    }
}

// 验证帧是否有效，返回 true = 有效，false = 无效（丢弃）
bool validate_frame(struct v4l2_buffer* buf, int width, int height, uint32_t pixelformat) {
    // 1. 检查驱动报告的错误标志
    if (buf->flags & V4L2_BUF_FLAG_ERROR) {
        printf("[CAPTURE] WARNING: frame flag error, skipping\n");
        return false;
    }

    // 2. 检查 bytesused 是否合理
    size_t expected = calculate_expected_size(width, height, pixelformat);
    size_t actual = buf->m.planes[0].bytesused;

    // 允许 10% 的误差范围
    if (expected > 0 && actual < expected * 0.9) {
        printf("[CAPTURE] WARNING: bytesused mismatch! expected=%zu, actual=%zu, skipping\n", expected, actual);
        return false;
    }

    return true;
}

// 自动检测可用的 ISP 设备节点
// 遍历 /dev/video0 ~ /dev/video31，查找 rkisp 驱动且未被独占的设备
const char* find_available_isp_device() {
    char device_path[64];
    static char found_path[64];

    for (int i = 0; i < 32; i++) {
        snprintf(device_path, sizeof(device_path), "/dev/video%d", i);

        // 尝试打开设备
        int fd = open(device_path, O_RDWR);
        if (fd < 0) {
            continue;
        }

        // 查询设备能力
        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            close(fd);
            continue;
        }

        // 检查是否为 rkisp 驱动
        if (strstr((const char*)cap.driver, "rkisp") == NULL) {
            close(fd);
            continue;
        }

        // 尝试申请 buffer 确认未被独占
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            close(fd);
            continue;
        }

        // 获取当前格式，检查分辨率是否为 576x324
        struct v4l2_format check_fmt;
        memset(&check_fmt, 0, sizeof(check_fmt));
        check_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        check_fmt.fmt.pix_mp.num_planes = 1;

        if (ioctl(fd, VIDIOC_G_FMT, &check_fmt) < 0) {
            close(fd);
            continue;
        }

        if (check_fmt.fmt.pix_mp.width != 576 || check_fmt.fmt.pix_mp.height != 324) {
            printf("Skipping %s: resolution %dx%d (need 576x324)\n",
                   device_path,
                   check_fmt.fmt.pix_mp.width,
                   check_fmt.fmt.pix_mp.height);
            close(fd);
            continue;
        }

        // 找到可用设备
        close(fd);
        snprintf(found_path, sizeof(found_path), "/dev/video%d", i);
        printf("Found available ISP device: %s (driver: %s)\n", found_path, cap.driver);
        return found_path;
    }

    return NULL;
}

// Capture Thread（生产者）：负责 V4L2 采集
void* capture_thread_func(void* arg) {
    // 获取线程上下文
    struct capture_context* ctx = static_cast<struct capture_context*>(arg);
    int fd = ctx->fd;
    struct buffer* buffers = ctx->buffers;
    int width = ctx->width;
    int height = ctx->height;
    uint32_t pixelformat = ctx->pixelformat;
    size_t frame_size = ctx->frame_size;

    struct pollfd fds;
    fds.fd = fd;
    fds.events = POLLIN;

    const int poll_timeout_ms = 2000;
    const int max_retries = 3;
    int retry_count = 0;
    int frame_id = 0;

    // 线程间通讯没有断，就不断去监听有没有拍到数据
    while (!g_shutdown_requested.load()) {
        int poll_ret = poll(&fds, 1, poll_timeout_ms);

        if (poll_ret < 0) {
            perror("[CAPTURE] poll");
            break;
        } else if (poll_ret == 0) {
            // 摄像头出现异常
            retry_count++;
            printf("[CAPTURE] poll timeout, retry %d/%d\n", retry_count, max_retries);
            if (retry_count >= max_retries) {
                printf("[CAPTURE] Max retries reached, exiting\n");
                break;
            }
            continue;
        }
        // 没有问题或者问题解决，就恢复
        retry_count = 0;

        // 每次发送数据前，先把buffer的旧数据清空
        struct v4l2_buffer buf;
        struct v4l2_plane planes[1];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1;

        // 丢入已完成队列
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            perror("[CAPTURE] VIDIOC_DQBUF");
            break;
        }

        frame_id++;  // 帧数++

        // 验证帧是否有效
        if (!validate_frame(&buf, width, height, pixelformat)) {
            // 帧无效，重新入队并继续
            if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                perror("[CAPTURE] VIDIOC_QBUF");
                break;
            }
            continue;  // 跳过这帧，继续下一轮
        }

        // 构建 Frame 对象，本质就是copy
        Frame frame;
        frame.data.resize(buf.m.planes[0].bytesused);
        memcpy(frame.data.data(), buffers[buf.index].start, buf.m.planes[0].bytesused);
        frame.bytesused = buf.m.planes[0].bytesused;
        frame.width = width;
        frame.height = height;
        frame.pixelformat = pixelformat;
        frame.frame_id = frame_id;

        // 丢入驱动可写队列
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("[CAPTURE] VIDIOC_QBUF");
            break;
        }

        // 入队（队列满时阻塞）
        if (!g_frame_queue.enqueue(std::move(frame))) {
            printf("[CAPTURE] queue stop signaled, exiting\n");
            break;
        }
    }

    printf("[CAPTURE] Thread exiting\n");
    return nullptr;
}

// Process Thread（消费者）：负责图像处理
void* process_thread_func(void* arg) {
    // 获取格式
    struct process_context* ctx = static_cast<struct process_context*>(arg);
    int yuv2bgr_code = ctx->yuv2bgr_code;

    // 获取 Frame 对象
    Frame frame;
    int save_counter = 0;

    while (g_frame_queue.dequeue(frame)) {

        int y_rows, y_cols;
        switch (frame.pixelformat) {
            case V4L2_PIX_FMT_NV12:
            case V4L2_PIX_FMT_NV21:
                y_rows = frame.height + frame.height / 2;
                y_cols = frame.width;
                break;
            case V4L2_PIX_FMT_YUYV:
            case V4L2_PIX_FMT_UYVY:
            case V4L2_PIX_FMT_YVYU:
                y_rows = frame.height;
                y_cols = frame.width * 2;
                break;
            default:
                fprintf(stderr, "[PROCESS] Unsupported pixel format: 0x%x\n", frame.pixelformat);
                continue;
        }

        cv::Mat yuv(y_rows, y_cols, CV_8UC1, frame.data.data());
        cv::Mat bgr;
        cv::cvtColor(yuv, bgr, yuv2bgr_code);

        // BGR → HSV 转换
        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        // 红色 HSV 阈值（阴影反光鲁棒）
        // 提高S最小值到150，排除低饱和度的阴影皮肤
        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv, cv::Scalar(0, 150, 120), cv::Scalar(15, 255, 255), mask1);   // 橙红区域
        cv::inRange(hsv, cv::Scalar(165, 150, 120), cv::Scalar(180, 255, 255), mask2);  // 深红区域
        cv::bitwise_or(mask1, mask2, mask);

        // 形态学操作：先开后闭一次
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);    // 去噪点
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);   // 填坑

        // 找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 用圆形度筛选候选者
        const double MIN_BALL_AREA = 1200;  // 最小面积阈值，过滤噪音
        const double CIRCULARITY_THRESHOLD = 0.5;  // 圆形度阈值，越接近1越圆
        std::vector<std::vector<cv::Point>> circularCandidates;  // 有圆形的候选者
        std::vector<double> circularAreas;  // 对应面积
        double maxArea = 0;
        int maxIdx = -1;
        bool hasCircle = false;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area < MIN_BALL_AREA) continue;

            double perimeter = cv::arcLength(contours[i], true);
            if (perimeter < 1e-6) continue;

            // 计算圆形度：4π×面积/周长²
            double circularity = 4 * CV_PI * area / (perimeter * perimeter);

            if (circularity >= CIRCULARITY_THRESHOLD) {
                // 是圆形，收集为候选
                circularCandidates.push_back(contours[i]);
                circularAreas.push_back(area);
                hasCircle = true;
            }

            // 记录最大面积（用于无圆时的回退逻辑）
            if (area > maxArea) {
                maxArea = area;
                maxIdx = i;
            }
        }

        // 根据是否有圆来选择目标
        if (hasCircle) {
            // 有圆，选圆中面积最大的
            double maxCircularArea = 0;
            int maxCircularIdx = -1;
            for (size_t i = 0; i < circularCandidates.size(); i++) {
                if (circularAreas[i] > maxCircularArea) {
                    maxCircularArea = circularAreas[i];
                    maxCircularIdx = i;
                }
            }
            maxIdx = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                if (cv::contourArea(contours[i]) == maxCircularArea) {
                    maxIdx = i;
                    break;
                }
            }
            maxArea = maxCircularArea;
        }

        // 质心坐标
        int cx = -1, cy = -1;
        if (maxIdx >= 0 && maxArea >= MIN_BALL_AREA) {
            cv::Moments m = cv::moments(contours[maxIdx]);
            cx = static_cast<int>(m.m10 / m.m00);
            cy = static_cast<int>(m.m01 / m.m00);
            printf("[CENTROID] (%d, %d) area=%.1f\n", cx, cy, maxArea);
        } else {
            printf("[CENTROID] not found\n");
        }

        // 每30帧保存原图并在质心处画准心
        save_counter++;
        if (save_counter % 30 == 0 && cx >= 0 && cy >= 0) {
            cv::Scalar color(0, 255, 0); // 绿色准心
            int crossSize = 15;
            cv::line(bgr, cv::Point(cx - crossSize, cy), cv::Point(cx + crossSize, cy), color, 2);
            cv::line(bgr, cv::Point(cx, cy - crossSize), cv::Point(cx, cy + crossSize), color, 2);
            cv::circle(bgr, cv::Point(cx, cy), 5, color, 2);

            // 添加 TARGET LOCKED 文字
            cv::putText(bgr, "TARGET LOCKED", cv::Point(cx + 20, cy - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

            char original_filename[64];
            snprintf(original_filename, sizeof(original_filename), "original_%05d.jpg", frame.frame_id);
            cv::imwrite(original_filename, bgr);
        }
    }

    printf("[PROCESS] Thread exiting\n");
    return nullptr;
}

int main()
{
    // 1 查找可用的 ISP 设备
    const char* device = find_available_isp_device();
    if (device == NULL) {
        fprintf(stderr, "No available ISP device found\n");
        return -1;
    }

    // 2 open
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("open");
        return -1;
    }
    printf("camera opened: %s\n", device);

    // 3 VIDIOC_QUERYCAP：查询设备能力
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        close(fd);
        return -1;
    }

    printf("driver: %s\n", cap.driver);
    printf("card: %s\n", cap.card);

    // 4 VIDIOC_G_FMT：获取当前格式（不重新设置，因该节点已被其他进程配置）
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;      // 格式选择和影像有关的
    fmt.fmt.pix_mp.num_planes = 1;                      // 和格式相关

    if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("VIDIOC_G_FMT");
        close(fd);
        return -1;
    }

    // 输出格式
    printf("width: %d\n", fmt.fmt.pix_mp.width);
    printf("height: %d\n", fmt.fmt.pix_mp.height);
    printf("pixelformat: 0x%x\n", fmt.fmt.pix_mp.pixelformat);

    // 根据 pixelformat 选择 YUV→BGR 转换码
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

    // 5 VIDIOC_REQBUFS：申请 buffer
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));

    req.count = 4;                                  // 申请4个 buffer
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;  // 格式选择和影像有关的
    req.memory = V4L2_MEMORY_MMAP;                  // 采用 mmap

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        close(fd);
        return -1;
    }

    struct buffer buffers[4];

    // 6 + 7 VIDIOC_QUERYBUF(查询 buffer 信息) + mmap(映射 buffer)
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

        if (buffers[i].start == MAP_FAILED) {
            perror("mmap");
            // 回滚前面已 mmap 的 buffer，释放映射内存
            for (int j = 0; j < i; j++) {
                munmap(buffers[j].start, buffers[j].length);
            }
            close(fd);
            return -1;
        }

        printf("buffer %d mapped\n", i);
    }

    // 8 VIDIOC_QBUF：全部入队
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

    // 9 VIDIOC_STREAMON：开始采集，不是真的采集是告诉驱动该采集了
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        close(fd);
        return -1;
    }

    printf("streaming...\n");

    // 注册 SIGINT handler，确保 Ctrl+C 时能正确清理资源
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, NULL) < 0) {
        perror("sigaction");
        close(fd);
        return -1;
    }

    // 准备线程上下文
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

    struct process_context proc_ctx = {
        .yuv2bgr_code = yuv2bgr_code
    };

    // 创建捕获线程和处理线程
    std::thread capture_thread(capture_thread_func, &cap_ctx);
    std::thread process_thread(process_thread_func, &proc_ctx);

    // 等待捕获线程结束（Ctrl+C 或采集出错时会触发）
    capture_thread.join();

    // 通知处理线程停止（处理完队列中剩余帧后退出）
    g_frame_queue.stop();

    // 等待处理线程结束
    process_thread.join();

    // 11 VIDIOC_STREAMOFF：关闭视频流
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("VIDIOC_STREAMOFF");
        close(fd);
        return -1;
    }

    // 12 munmap：释放映射内存
    for (int i = 0; i < 4; i++) {
        if (munmap(buffers[i].start, buffers[i].length) < 0) {
            perror("munmap");
        }
    }

    // 关闭文件描述符
    close(fd);
    return 0;
}
