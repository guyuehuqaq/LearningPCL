cmake_minimum_required(VERSION 3.10)

# 用octree对点云数据进行压缩, 从点云设备中提取点云数据，压缩并可视化
project(PointCloudCompression)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(PointCloudCompression point_cloud_compress.cpp)
find_package(PCL REQUIRED)
target_link_libraries(PointCloudCompression PRIVATE ${PCL_LIBRARIES})
