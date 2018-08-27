cmake_minimum_required( VERSION 2.8 )
project( Disparity )


set( CMAKE_CXX_FLAGS "--std=c++11")

include_directories( "/usr/include/eigen3" )

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( disparity  disparity.cpp)

target_link_libraries(disparity ${OpenCV_LIBS})
target_link_libraries( disparity ${Pangolin_LIBRARIES} ) 
