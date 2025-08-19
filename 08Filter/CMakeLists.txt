cmake_minimum_required(VERSION 3.10)
project(Filter)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED)

add_executable(pass_through_filter pass_through_filter.cpp)
target_link_libraries(pass_through_filter ${PCL_LIBRARIES})
