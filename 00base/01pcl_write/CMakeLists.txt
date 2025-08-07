cmake_minimum_required(VERSION 3.15)
project(pcl_write)

# 指定 C++ 版本
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(pcl_write pcl_write.cpp)
target_link_libraries(pcl_write PRIVATE ${PCL_LIBRARIES})