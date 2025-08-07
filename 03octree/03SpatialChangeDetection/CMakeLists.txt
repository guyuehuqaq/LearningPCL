cmake_minimum_required(VERSION 3.10)

# 用octree对点云数据进行压缩, 从点云设备中提取点云数据，压缩并可视化
project(OCTree)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Spatial Partitioning and Search Operations with Octrees
# pcl用octree实现空间区和邻居搜索
add_executable(octree_change_detection octree_change_detection.cpp)
find_package(PCL CONFIG REQUIRED)
target_link_libraries(octree_change_detection PRIVATE ${PCL_LIBRARIES})