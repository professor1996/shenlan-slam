cmake_minimum_required( VERSION 2.8 )
project( gaussnewton )


set( CMAKE_CXX_FLAGS "--std=c++11")

include_directories( "/usr/include/eigen3" )

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( gaussnewton  gaussnewton.cpp)

target_link_libraries(gaussnewton ${OpenCV_LIBS})
 
