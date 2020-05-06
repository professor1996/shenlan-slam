cmake_minimum_required(VERSION 2.8)
project(img)
include_directories(include)
find_package(OpenCV  REQUIRED)


add_executable(img main.cpp RMVideoCapture.cpp)
target_link_libraries(img ${OpenCV_LIBS})
