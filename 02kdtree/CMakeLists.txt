cmake_minimum_required(VERSION 3.10)

project(kdtree)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#file(GLOB basic_srcs "*.cpp" "*.h")
add_executable(kdtree_flann kdtree_flann.cpp
        kdtree_flann.h)
add_executable(kdtree_nanoflann kdtree_nanoflann.cpp)
add_executable(kdtree_pcl kdtree_pcl.cpp)

find_package(flann CONFIG REQUIRED)
target_link_libraries(kdtree_flann PRIVATE flann::flann_s flann_cpp_s)

find_package(nanoflann CONFIG REQUIRED)
target_link_libraries(kdtree_nanoflann PRIVATE nanoflann::nanoflann)

find_package(PCL REQUIRED)
target_include_directories(kdtree_pcl PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(kdtree_pcl PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(kdtree_pcl PRIVATE ${PCL_DEFINITIONS})



