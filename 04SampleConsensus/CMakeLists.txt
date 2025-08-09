cmake_minimum_required(VERSION 3.10)

# 用octree对点云数据进行压缩, 从点云设备中提取点云数据，压缩并可视化
project(PCL)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 用pcl中定义好的模型
add_executable(pcl_ransac pcl_ransac.cpp)
# 用自己定义的模型
add_executable(pcl_ransac_1 pcl_ransac_1.cpp)
find_package(PCL REQUIRED)
target_link_libraries(pcl_ransac PRIVATE ${PCL_LIBRARIES})
target_link_libraries(pcl_ransac_1 PRIVATE ${PCL_LIBRARIES})