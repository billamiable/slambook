# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( useHelloSLAM )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( usehelloSLAM use_helloSLAM.cpp )

# 寻找
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(HELLO)
include_directories(${HELLO_INCLUDE_DIRS})
target_link_libraries (usehelloSLAM ${HELLO_LIBRARIES})