cmake_minimum_required(VERSION 3.10)

# 用octree对点云数据进行压缩, 从点云设备中提取点云数据，压缩并可视化
project(PCL_OI)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED)

add_executable(write_pcd write_pcd.cpp)
target_link_libraries(write_pcd PRIVATE ${PCL_LIBRARIES})

add_executable(read_pcd read_pcd.cpp)
target_link_libraries(read_pcd PRIVATE ${PCL_LIBRARIES})

add_executable(concatenate_cloud concatenate_cloud.cpp)
target_link_libraries(concatenate_cloud PRIVATE ${PCL_LIBRARIES})

add_executable(openni_viewer_simple openni_viewer_simple.cpp)
target_link_libraries(openni_viewer_simple PRIVATE ${PCL_LIBRARIES})