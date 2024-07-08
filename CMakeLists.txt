cmake_minimum_required(VERSION 3.10)

# 项目信息
project(MotionPlanning)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 包含目录
include_directories(
        D:/forprogram/MotionPlanning/ompl/build/src
        D:/forprogram/vcpkg-master/installed/x64-windows/include
        D:/forprogram/MotionPlanning/ompl/src
        D:/forprogram/Eigen3
)

# 链接库目录
link_directories(
        D:/forprogram/vcpkg-master/installed/x64-windows/lib
        D:/forprogram/MotionPlanning/ompl/build/src/ompl/Debug
)

# 添加源文件
add_executable(MotionPlanning main.cpp)

# 链接库
target_link_libraries(MotionPlanning
        D:/forprogram/MotionPlanning/ompl/build/src/ompl/Debug/ompl.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_program_options-vc144-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_serialization-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_filesystem-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_unit_test_framework-vc144-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_thread-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_chrono-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_date_time-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_atomic-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_system-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_container-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_regex-vc142-mt-x64-1_85.lib
        D:/forprogram/vcpkg-master/installed/x64-windows/lib/boost_exception-vc142-mt-x64-1_85.lib
)
# 设置 vcpkg 工具链文件路径
set(CMAKE_TOOLCHAIN_FILE "D:/forprogram/vcpkg-master/scripts/buildsystems/vcpkg.cmake")
