cmake_minimum_required(VERSION 3.10)
project(Segment)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED)

add_executable(planar_segmentation planar_segmentation.cpp)
target_link_libraries(planar_segmentation PRIVATE ${PCL_LIBRARIES})