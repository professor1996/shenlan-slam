cmake_minimum_required( VERSION 2.8 )
project( image_undistortion )


set( CMAKE_CXX_FLAGS "--std=c++11")

find_package( OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( undistort_image  undistort_image.cpp)

target_link_libraries(undistort_image ${OpenCV_LIBS})
