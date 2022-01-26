cmake_minimum_required (VERSION 2.8.9)
project (CBC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fPIC")  
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    include_directories(.)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message("${OpenCV_INCLUDE_DIRS}")
    
    add_library(CBC SHARED cbc.cpp cow_condition.cpp)
    target_link_libraries(CBC ${OpenCV_LIBRARIES})    

    add_executable(runner demo.cpp )
    target_link_libraries(runner CBC ${OpenCV_LIBRARIES})
endif()


