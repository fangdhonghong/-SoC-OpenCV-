#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

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
#include <cmath>

// ============ 全局变量 ============
extern volatile sig_atomic_t running;
extern std::atomic<bool> g_shutdown_requested;

// ============ Frame 结构体 ============
struct Frame {
    std::vector<uint8_t> data;
    size_t bytesused;
    int width, height;
    uint32_t pixelformat;
    int frame_id;
};

// ============ 线程安全队列模板 ============
template<typename T>
class ThreadSafeQueue {
    std::queue<T> queue_;           // 队列
    std::mutex mutex_;              // 互斥锁
    std::condition_variable cond_;  // 条件变量
    size_t max_size_;               // 最大容量
    bool stop_signaled_;            // 是否停止
public:
    explicit ThreadSafeQueue(size_t max_size);                     // 初始化
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;              // 禁止复制
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;   // 禁止把队列赋值给另一个队列
    bool enqueue(T&& frame);                                       // 入队
    bool dequeue(T& frame);                                        // 出队
    void stop();                                                   // 停止队列
    bool is_stopped() const;                                       // 查看是否停止
};

// ============ 线程安全队列的方法实现 ============

template<typename T>
ThreadSafeQueue<T>::ThreadSafeQueue(size_t max_size) : max_size_(max_size), stop_signaled_(false) {}  

template<typename T>
bool ThreadSafeQueue<T>::enqueue(T&& frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stop_signaled_) {              // 队列停止
        return false;
    }
    if (queue_.size() >= max_size_) {  // 队列满，移除尾数据
        queue_.pop();
    }
    queue_.push(std::move(frame));     // 将数据放入队列
    cond_.notify_one();                // 唤起一个线程
    return true;
}

template<typename T>
bool ThreadSafeQueue<T>::dequeue(T& frame) {
    std::unique_lock<std::mutex> lock(mutex_);  // 上锁
    if (stop_signaled_ && queue_.empty()) {     // 队列停止 并且 队列为空
        return false;
    }
    cond_.wait(lock, [this] { return !queue_.empty() || stop_signaled_; });  // 释放互斥锁 + 睡眠
    if (stop_signaled_ && queue_.empty()) {     // 队列停止 或 队列为空
        return false;
    }
    frame = std::move(queue_.front());          // 复制数据
    queue_.pop();                               // 出队列
    cond_.notify_one();                         // 唤起一个线程
    return true; 
}

template<typename T>
void ThreadSafeQueue<T>::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);  // 上锁
        stop_signaled_ = true;                     // 停止队列
    }
    cond_.notify_all();                            // 唤起所有线程
}

template<typename T>
bool ThreadSafeQueue<T>::is_stopped() const {
    std::lock_guard<std::mutex> lock(mutex_);     // 上锁
    return stop_signaled_;
}

// ============ 全局帧队列 ============
extern ThreadSafeQueue<Frame> g_frame_queue;     // 生产者和消费者之间的桥梁

// ============ 缓冲区结构体 ============
// 储存用户向V4L2申请的buffer
struct buffer {
    void *start;
    size_t length;
};

// ============ 线程上下文（生产者） ============
struct capture_context {
    int fd;
    struct buffer* buffers;
    int width;
    int height;
    uint32_t pixelformat;
    size_t frame_size;
};

// ============ 质心跟踪器 ============
struct CentroidTracker {
    int cx, cy;                    // 当前位置
    int prev_cx, prev_cy;          // 上一帧的位置
    int filtered_cx, filtered_cy;  // 滤波后的数据
    bool filter_initialized;       // 是否是第一次滤波
    float alpha;                   // 一阶低通滤波的系数
    float vx, vy;                  // 当前速度
    float max_delta_distance;      // 最大位移阈值
};

// ============ 决策树 ============
struct DecisionTree {
    bool identified;          // 是否识别
    int consecutive_misses;   // 连续丢帧数
    int max_misses;           // 最大丢帧数
    CentroidTracker tracker;  // 质心跟踪器
};

// ============ 线程上下文（消费者） ============
struct process_context {
    int yuv2bgr_code;
    DecisionTree decision;
};

// ============ 函数声明 ============
void sigint_handler(int sig);

size_t calculate_expected_size(int width, int height, uint32_t pixelformat);
bool validate_frame(struct v4l2_buffer* buf, int width, int height, uint32_t pixelformat);

const char* find_available_isp_device();
void* capture_thread_func(void* arg);

void decision_tree_update(DecisionTree* dt, int raw_cx, int raw_cy, int* output_cx, int* output_cy);
void* process_thread_func(void* arg);

#endif // V4L2_CAPTURE_H