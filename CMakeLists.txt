# 1.指定最低CMake版本
cmake_minimum_required(VERSION 3.10)

# 2.定义项目名称，并显式声明需要c 和 c++ 两种语言支持
project(STworld LANGUAGES C CXX)

# 3.指定c++标准
set(CMAKE_CXX_STANDARD 14)

# ==================== 核心修改点 ====================
# 判断是否为交叉编译（ARM 平台）
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    message(STATUS "Cross-compiling for ARM")

    # ARM 版 OpenCV 路径（opencv-mobile luckfox-pico）
    set(ARM_OPENCV_PATH "$ENV{HOME}/opencv-mobile-4.10.0-luckfox-pico")

    # 手动设置 OpenCV 变量（不使用 find_package）
    set(OpenCV_VERSION "4.10.0")
    set(OpenCV_INCLUDE_DIRS "${ARM_OPENCV_PATH}/include/opencv4")
    set(OpenCV_LIBRARY_DIRS "${ARM_OPENCV_PATH}/lib")

    # 静态库文件 - 使用正确的链接顺序
    # imgproc 在中间，core 在最后
    set(OpenCV_LIBS
        "-Wl,--whole-archive"
        "${ARM_OPENCV_PATH}/lib/libopencv_imgproc.a"
        "${ARM_OPENCV_PATH}/lib/libopencv_highgui.a"
        "${ARM_OPENCV_PATH}/lib/libopencv_core.a"
        "-Wl,--no-whole-archive"
    )

    message(STATUS "Using opencv-mobile from: ${ARM_OPENCV_PATH}")
else()
    # 本地编译使用 find_package
    set(OpenCV_DIR /root/luckfox_pico_rknn_example/lib/uclibc/lib/cmake/opencv4)
    find_package(OpenCV REQUIRED)
endif()
# ====================================================

# 打印一下找到的信息，方便你确认是否成功
message(STATUS "OpenCV found successfully!")
message(STATUS "    OpenCV version: ${OpenCV_VERSION}")
message(STATUS "    OpenCV include path: ${OpenCV_INCLUDE_DIRS}")

# 5.指定可执行文件的输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# 6. 生成可执行文件
add_executable(STworld v4l2_capture.cpp)

# 7.包含头文件目录
target_include_directories(STworld PRIVATE ${OpenCV_INCLUDE_DIRS})

# 8. 链接 OpenCV 库
if(CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    target_link_libraries(STworld ${OpenCV_LIBS})
else()
    target_link_libraries(STworld ${OpenCV_LIBS})
endif()