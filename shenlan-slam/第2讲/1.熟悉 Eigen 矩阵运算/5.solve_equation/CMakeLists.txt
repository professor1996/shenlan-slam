cmake_minimum_required( VERSION 2.8 )
project( useEigen )

set( CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-O3" )

include_directories( "/usr/include/eigen3" )
#include_directories( "/home/ubuntu/Downloads/eigen-eigen-5a0156e40feb/Eigen")

add_executable( Solve_Matrix-equation  Matrix_equation.cpp)
