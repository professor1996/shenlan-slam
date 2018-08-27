cmake_minimum_required( VERSION 2.8 )
project( direct_method )


set( CMAKE_CXX_FLAGS "--std=c++11")

include_directories( "/usr/include/eigen3" )

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
 
find_package( Sophus  )
include_directories( ${Sophus_INCLUDE_DIRS} )

#find_package( Pangolin )
#include_directories( ${Pangolin_INCLUDE_DIRS} )


add_executable(        direct_method  direct_method.cpp)
target_link_libraries( direct_method ${OpenCV_LIBS})
target_link_libraries( direct_method ${Sophus_LIBRARIES} )
#target_link_libraries( direct_method ${Pangolin_LIBRARIES} ) 
