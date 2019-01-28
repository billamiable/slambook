cmake_minimum_required( VERSION 2.8 )
project( ceres_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用ceres库
# 这个还挺神奇的哈哈！
# 原因是ceres不常用，要把怎么找它的方式加进来~
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 使用ceres有两步：1.include头文件；2. link编译好的library
# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( curve_fitting main.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( curve_fitting ${CERES_LIBRARIES} ${OpenCV_LIBS} )