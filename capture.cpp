#include "v4l2_capture.h"

// ============ 帧验证辅助函数 ============

/**
 * 预算 buffer 的长度
 * pixelformat：图片格式
 */
size_t calculate_expected_size(int width, int height, uint32_t pixelformat) {
    switch (pixelformat) {
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV21:
            return width * height * 3 / 2;
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_YVYU:
            return width * height * 2;
        default:
            return 0;
    }
}

/**
 * 验证帧是否合法
 * buf：用户向V4L2驱动申请的buffer
 * pixelformat：图像格式
 */
bool validate_frame(struct v4l2_buffer* buf, int width, int height, uint32_t pixelformat) {
    if (buf->flags & V4L2_BUF_FLAG_ERROR) {  // 检测到 V4L2 错误标志
        printf("[CAPTURE] WARNING: frame flag error, skipping\n");
        return false;
    }

    size_t expected = calculate_expected_size(width, height, pixelformat);  // 预测的长度
    size_t actual = buf->m.planes[0].bytesused;                             // 驱动给的长度

    if (expected > 0 && actual < expected * 0.9) {  // 驱动给的长度不足预期值的90%，判定为不完整帧，丢弃
        printf("[CAPTURE] WARNING: bytesused mismatch! expected=%zu, actual=%zu, skipping\n", expected, actual);
        return false;
    }

    return true;
}



// 生产者
void* capture_thread_func(void* arg) {
    // 获取线程上下文
    struct capture_context* ctx = static_cast<struct capture_context*>(arg);
    int fd = ctx->fd;
    struct buffer* buffers = ctx->buffers;
    int width = ctx->width;
    int height = ctx->height;
    uint32_t pixelformat = ctx->pixelformat;

    // poll
    struct pollfd fds;
    fds.fd = fd;
    fds.events = POLLIN;  // 有数据可读时唤起

    const int poll_timeout_ms = 2000;  // 阻塞时长
    const int max_retries = 3;         // 最大重试机会
    int retry_count = 0;               // 重试次数
    int frame_id = 0;                  // 帧数

    while (!g_shutdown_requested.load()) {  // shutdown_requested不为0
        int poll_ret = poll(&fds, 1, poll_timeout_ms);  // 阻塞等待，有数据时唤起

        if (poll_ret < 0) {
            perror("[CAPTURE] poll");
            break;
        } else if (poll_ret == 0) {  // poll 超时：摄像头异常
            retry_count++;  // 重试次数++
            printf("[CAPTURE] poll timeout, retry %d/%d\n", retry_count, max_retries);
            if (retry_count >= max_retries) {
                printf("[CAPTURE] Max retries reached, exiting\n");  // 超过最大重试机会直接退出
                break;
            }
            continue;
        }
        retry_count = 0;  // 重试次数清空

        // 开始实时流：从驱动中取出一帧 -> copy -> 还给驱动 -> 交给消费者
        // 每一次循环都要初始化buffer，防止上一次数据的影响
        struct v4l2_buffer buf;
        struct v4l2_plane planes[1];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1;

        // 从驱动中取出一帧
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            perror("[CAPTURE] VIDIOC_DQBUF");
            break;
        }

        frame_id++;

        // 检测该帧的数据是否合法
        if (!validate_frame(&buf, width, height, pixelformat)) {
            if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {  // 不合法：把该帧数据还给buffer
                perror("[CAPTURE] VIDIOC_QBUF");
                break;
            }
            continue;
        }

        // copy
        Frame frame;
        frame.data.resize(buf.m.planes[0].bytesused);
        memcpy(frame.data.data(), buffers[buf.index].start, buf.m.planes[0].bytesused);
        frame.bytesused = buf.m.planes[0].bytesused;
        frame.width = width;
        frame.height = height;
        frame.pixelformat = pixelformat;
        frame.frame_id = frame_id;

        // 还给驱动
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("[CAPTURE] VIDIOC_QBUF");
            break;
        }

        // 交给消费者
        if (!g_frame_queue.enqueue(std::move(frame))) {
            printf("[CAPTURE] queue stop signaled, exiting\n");
            break;
        }
    }

    return nullptr;
}