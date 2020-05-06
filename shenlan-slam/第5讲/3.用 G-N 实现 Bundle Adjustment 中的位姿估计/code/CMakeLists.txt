cmake_minimum_required( VERSION 2.8 )
project( GN-BA )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

 

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( GN-BA GN-BA.cpp )
# 链接OpenCV库
target_link_libraries( GN-BA ${Sophus_LIBRARIES} )
