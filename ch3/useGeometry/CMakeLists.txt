cmake_minimum_required( VERSION 2.8 )
project( geometry )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# in osx and brew install
# include_directories( /usr/local/Cellar/eigen/3.3.5/include/eigen3 )

add_executable( eigenGeometry eigenGeometry.cpp )
