cmake_minimum_required( VERSION 2.8 )
project( ICP )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

include_directories( "/usr/include/eigen3" )

find_package( Sophus  )
include_directories( ${Sophus_INCLUDE_DIRS} )

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )
add_executable( ICP ICP.cpp )
# 链接OpenCV库
target_link_libraries( ICP ${Sophus_LIBRARIES} )
target_link_libraries( ICP ${Pangolin_LIBRARIES} ) 
