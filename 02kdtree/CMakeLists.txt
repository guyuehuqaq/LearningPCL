cmake_minimum_required(VERSION 3.10)

project(kdtree_search)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB basic_srcs "*.cpp" "*.h")
add_executable(kdtree_search ${basic_srcs})

find_package(nanoflann CONFIG REQUIRED)
include_directories(${MANOFLANN_DIRS})

find_package(PCL REQUIRED)
target_include_directories(kdtree_search PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(kdtree_search PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(kdtree_search PRIVATE ${PCL_DEFINITIONS})



