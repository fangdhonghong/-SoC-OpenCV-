#include "v4l2_capture.h"

/**
 * 决策树
 * dt：决策树结构体
 * raw_cx，raw_cy：原始质心
 * output_cx，output_cy：经过滤波的质心
 */
void decision_tree_update(DecisionTree* dt, int raw_cx, int raw_cy, int* output_cx, int* output_cy) {
    *output_cx = -1;
    *output_cy = -1;

    if (raw_cx >= 0 && raw_cy >= 0) { // 识别到质心
        float delta_d = 0.0f;
        if (dt->identified && dt->tracker.prev_cx >= 0 && dt->tracker.prev_cy >= 0) { // 已识别且捕获到上一帧
            float dx = raw_cx - dt->tracker.prev_cx;
            float dy = raw_cy - dt->tracker.prev_cy;
            delta_d = sqrtf(dx * dx + dy * dy);  // 质心移动的长度
        }

        if (dt->identified && delta_d > dt->tracker.max_delta_distance) { // 已识别且质心移动位移大于阈值，视为丢帧
            printf("[DECISION] Skip: Δd=%.1f > %.1f (false positive)\n",
                   delta_d, dt->tracker.max_delta_distance);
            dt->consecutive_misses++;
        } else {
            // 更新当前位置和上一帧位置
            dt->tracker.prev_cx = dt->tracker.cx;
            dt->tracker.prev_cy = dt->tracker.cy;
            dt->tracker.cx = raw_cx;
            dt->tracker.cy = raw_cy;

            if (dt->identified && delta_d > 0) { // 已识别且移动长度合法，更新速度
                dt->tracker.vx = dt->tracker.cx - dt->tracker.prev_cx;
                dt->tracker.vy = dt->tracker.cy - dt->tracker.prev_cy;
            }

            // 开始滤波
            if (!dt->tracker.filter_initialized) {
                dt->tracker.filtered_cx = dt->tracker.cx;
                dt->tracker.filtered_cy = dt->tracker.cy;
                dt->tracker.filter_initialized = true;
                printf("[DECISION] Initialized: (%d, %d)\n", dt->tracker.filtered_cx, dt->tracker.filtered_cy);
            } else {
                dt->tracker.filtered_cx = static_cast<int>(dt->tracker.alpha * dt->tracker.cx +
                    (1.0f - dt->tracker.alpha) * dt->tracker.filtered_cx);
                dt->tracker.filtered_cy = static_cast<int>(dt->tracker.alpha * dt->tracker.cy +
                    (1.0f - dt->tracker.alpha) * dt->tracker.filtered_cy);
            }

            // 更新滤波后的值
            *output_cx = dt->tracker.filtered_cx;
            *output_cy = dt->tracker.filtered_cy;
            dt->consecutive_misses = 0;  // 无丢帧
            dt->identified = true;       // 已识别
            printf("[DECISION] filtered=(%d,%d)\n", *output_cx, *output_cy);  // 打印质心
        }
    } else {  // 未识别
        dt->consecutive_misses++;  // 更新丢帧数
        printf("[DECISION] Miss #%d/%d\n", dt->consecutive_misses, dt->max_misses);

        if (dt->consecutive_misses <= dt->max_misses) {  // 连续丢帧数小于设定阈值，用速度预测位置
            *output_cx = dt->tracker.filtered_cx + static_cast<int>(dt->tracker.vx);
            *output_cy = dt->tracker.filtered_cy + static_cast<int>(dt->tracker.vy);
            printf("[DECISION] Predicted: (%d, %d) + v=(%.1f, %.1f)\n",
                   dt->tracker.filtered_cx, dt->tracker.filtered_cy, dt->tracker.vx, dt->tracker.vy);
        } else {  // 连续丢帧数大于设定阈值，目标丢失
            dt->identified = false;  // 标识丢失目标
            printf("[DECISION] Lost target after %d misses\n", dt->consecutive_misses);
        }
    }
}

// 消费者
void* process_thread_func(void* arg) {
    // 获取上下文线程
    struct process_context* ctx = static_cast<struct process_context*>(arg);
    int yuv2bgr_code = ctx->yuv2bgr_code;

    // 获取生产者发来的队列
    Frame frame;
    int save_counter = 0;

    while (g_frame_queue.dequeue(frame)) {  // 队列有数据

        // 根据格式获取参数值
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

        // 解释为图像
        cv::Mat yuv(y_rows, y_cols, CV_8UC1, frame.data.data());
        cv::Mat bgr;
        cv::cvtColor(yuv, bgr, yuv2bgr_code);

        // 获取HSV
        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        // 获取mask
        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv, cv::Scalar(0, 150, 120), cv::Scalar(15, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(165, 150, 120), cv::Scalar(180, 255, 255), mask2);
        cv::bitwise_or(mask1, mask2, mask);

        // 形态学滤波
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 获取轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 获取目标
        const double MIN_BALL_AREA = 1200;  // 最小面积阈值
        const double CIRCULARITY_THRESHOLD = 0.5;  // 圆形度的值，越接近1越圆，我们的候选者只要求1/2
        std::vector<std::vector<cv::Point>> circularCandidates;
        std::vector<double> circularAreas;
        double maxArea = 0;  // 轮廓面积
        int maxIdx = -1;     // 当前面积最大值
        bool hasCircle = false;

        // 循环遍历轮廓
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);  // 获取轮廓面积
            if (area < MIN_BALL_AREA) continue;          // 小于最小阈值就淘汰

            double perimeter = cv::arcLength(contours[i], true);  // 获取轮廓周长
            if (perimeter < 1e-6) continue;                       // 周长过小就淘汰

            double circularity = 4 * CV_PI * area / (perimeter * perimeter);  // 获取轮廓的圆形度的值

            if (circularity >= CIRCULARITY_THRESHOLD) {  // 有像圆的趋势
                circularCandidates.push_back(contours[i]);
                circularAreas.push_back(area);
                hasCircle = true;  // 在像圆这个决策路径中有候选者
            }

            if (area > maxArea) {  // 选出面积最大的一个
                maxArea = area;
                maxIdx = i;
            }
        }

        if (hasCircle) {  // 如果在像圆这个决策路径中有候选者
            double maxCircularArea = 0;  // 最大轮廓面积
            int maxCircularIdx = -1;     // 最大圆形度的值
            for (size_t i = 0; i < circularCandidates.size(); i++) {   // 遍历所有候选，选出最像圆的那一个
                if (circularAreas[i] > maxCircularArea) {
                    maxCircularArea = circularAreas[i];
                    maxCircularIdx = i;
                }
            }
            maxIdx = -1;  // 把面积最大的一个id清空
            for (size_t i = 0; i < contours.size(); i++) {  // 在最像圆的那一批候选者中，选出面积最大的那一个
                if (cv::contourArea(contours[i]) == maxCircularArea) {
                    maxIdx = i;
                    break;
                }
            }
            maxArea = maxCircularArea;
        }

        // 开始找质心
        int raw_cx = -1, raw_cy = -1;

        // 获取质心原始值
        if (maxIdx >= 0 && maxArea >= MIN_BALL_AREA) {
            cv::Moments m = cv::moments(contours[maxIdx]);
            raw_cx = static_cast<int>(m.m10 / m.m00);
            raw_cy = static_cast<int>(m.m01 / m.m00);
        }

        // 更新决策
        int output_cx = -1, output_cy = -1;
        decision_tree_update(&ctx->decision, raw_cx, raw_cy, &output_cx, &output_cy);

        save_counter++;  // 获取帧数
        if (save_counter % 30 == 0 && output_cx >= 0 && output_cy >= 0) {  // 当帧数到达30帧并且找到质心
            // 在质心上画准心
            cv::Scalar color(0, 255, 0);
            int crossSize = 15;
            cv::line(bgr, cv::Point(output_cx - crossSize, output_cy), cv::Point(output_cx + crossSize, output_cy), color, 2);
            cv::line(bgr, cv::Point(output_cx, output_cy - crossSize), cv::Point(output_cx, output_cy + crossSize), color, 2);
            cv::circle(bgr, cv::Point(output_cx, output_cy), 5, color, 2);

            // 打印文字
            cv::putText(bgr, "TARGET LOCKED", cv::Point(output_cx + 20, output_cy - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

            // 保存图片
            char original_filename[64];
            snprintf(original_filename, sizeof(original_filename), "original_%05d.jpg", frame.frame_id);
            cv::imwrite(original_filename, bgr);
        }
    }

    printf("[PROCESS] Thread exiting\n");
    return nullptr;
}