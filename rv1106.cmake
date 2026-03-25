# 1. 操作系统名称
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 2. 指定交叉编译器路径 (根据你的实际路径修改)
set(TOOLCHAIN_PATH "/root/luckfox-pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf")
set(CMAKE_C_COMPILER   "${TOOLCHAIN_PATH}/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/bin/arm-rockchip830-linux-uclibcgnueabihf-g++")

# 3. 指定 Sysroot (非常重要：防止它去找宿主机的头文件)
# 通常 sysroot 在工具链目录下的某个子目录
set(CMAKE_SYSROOT "${TOOLCHAIN_PATH}/arm-rockchip830-linux-uclibcgnueabihf/sysroot")

# 4. 设置寻找库和程序的搜索模式
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)