cmake_minimum_required( VERSION 2.8 )
project( E2Rt )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 3.1 REQUIRED )
# 添加头文件

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( E2Rt E2Rt.cpp )
# 链接OpenCV库
target_link_libraries( E2Rt ${Sophus_LIBRARIES} )
