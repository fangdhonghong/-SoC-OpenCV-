# V4L2 Rockchip ISP 摄像头采集与 OpenCV 目标跟踪


本项目是一个基于 Linux V4L2 (Video for Linux 2) API 和 OpenCV 的 C++ 实时图像采集与处理程序。它专门针对 瑞芯微 (Rockchip) 平台的 ISP 摄像头设计，采用多线程生产者-消费者模型，实现了低延迟的红色圆形目标（如红球）实时检测与跟踪。

## 主要特性
硬件级自动适配：自动遍历 /dev/video* 节点，寻找未被独占的 rkisp 驱动设备。

V4L2 M-Plane 支持：使用 V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE 多平面 API 获取图像。

高性能多线程架构：

Capture Thread（生产者）：负责与底层 V4L2 驱动交互，零拷贝映射（mmap）抓取图像。

Process Thread（消费者）：负责使用 OpenCV 进行图像处理。

防阻塞机制：自定义线程安全队列，在队列满时自动丢弃最老帧，保证实时视频流不产生延迟积压。

目标检测：
基于双重 HSV 颜色阈值过滤（适应高光与阴影环境中的红色）。

结合形态学操作（开运算、闭运算）去噪。

结合轮廓面积与圆形度 (Circularity) 双重校验，精准锁定圆形目标。

优雅退出：捕获 Ctrl+C (SIGINT) 信号，安全释放 V4L2 缓冲区和内存映射。

## 硬件 效果示意
![990904a9ff53c3c76d7fd8c602c2dbd5](https://github.com/user-attachments/assets/42a063d4-656e-4ae4-bb95-22f507aed78e)


<img width="1479" height="759" alt="image" src="https://github.com/user-attachments/assets/af30be06-ce2d-4614-81d1-3973f3e2a9e3" />

<img width="1920" height="1020" alt="aa52232435a47846acd08327fa2db822" src="https://github.com/user-attachments/assets/c88baae4-3a73-4dcb-a91b-55d64e8da0e1" />


## 架构示意图

│ ▼ mmap / DQBUF (Capture Thread - 生产者)

│ ▼ 写入 std::move(Frame)[ ThreadSafeQueue (容量:4, 满则丢弃队首) ]

│ ▼ 阻塞读取 (Process Thread - 消费者)

│ ├─ YUV -> BGR -> HSV

│ ├─ inRange (颜色过滤)

│ ├─ 形态学去噪

│ ├─ findContours

│ ├─ 计算面积与圆形度过滤

│ └─ 保存图像 (TARGET LOCKED)


# 新增

1.消费者更新：

消费者原逻辑是：取一帧照片 -> HSV -> mask -> 形态学滤波 -> 找轮廓 -> 输出质心

更新后的逻辑：取一帧照片 -> HSV -> mask -> 形态学滤波 -> 找轮廓 ->决策 -> 输出质心

决策树

                ┌────────────────────┐
                │  目标是否存在？     │
                └──────┬─────────────┘
                       │
        ┌──────────────┴──────────────┐
        │                             │

    否（没检测到）                是（检测到了）
        │                             │
        ↓                             ↓
   连续丢帧数量++               计算 Δd = 与上一帧距离
        │                             ↓
        │                    ┌────────────────────┐
        │                    │ Δd 是否合理？      │
        │                    └──────┬─────────────┘
        │                           │
        │                   否（跳太远）    是（正常）
        │                           │
        │                           ↓
        │                     接受检测结果
        │                     更新真实位置(cx,cy)
        │                     更新速度(vx,vy)
        │                     连续丢帧量 = 0
        │                     标识已识别
        │
        ↓
┌────────────────────┐
│  连续丢帧量 ≤ N ?  │
└──────┬─────────────┘
       │
    是（短暂丢失）          否（彻底丢失）
       │                      │
       ↓                      ↓
输出预测位置（不写回）       identified = 0
(cx + vx, cy + vy)           不再输出目标

2.解耦

把原来的扁平结构变成嵌套结构

v4l2_capture.h：函数声明 和 全局变量 和 定义体

v4l2_capture.cpp：主线程（main）

capture.cpp：生产者线程（Capture Thread）

decision.cpp：消费者线程（Process Thread）



# 测试
再光线正常（无逆光，光线够）且无高速无规则乱动的情况下，运行了12905帧，每30帧保存数据并再质心画准心，在保存的图片中准心都画对了

在逆光或暗情况无法正常检测，高速无规则乱动的情况下无法预测预测其跑出视野后的大概方向
