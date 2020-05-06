#PROJECT(helloSLAM)



#ADD_LIBRARY(hello  src/hello.cpp)




#ADD_EXECUTABLE(hiSLAM  src/useHello.cpp)


#TARGET_LINK_LIBRARIES(hiSLAM  hello)












PROJECT(helloSLAM)

set(CMAKE_BUILD_TYPE "Release")



ADD_LIBRARY(${PROJECT_NAME}  src/hello.cpp)




ADD_EXECUTABLE(hiSLAM  
src/useHello.cpp)


TARGET_LINK_LIBRARIES(hiSLAM  ${PROJECT_NAME})
