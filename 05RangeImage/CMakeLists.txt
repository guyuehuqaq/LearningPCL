cmake_minimum_required(VERSION 3.10)

# 用octree对点云数据进行压缩, 从点云设备中提取点云数据，压缩并可视化
project(RangeImage)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED)

# range image是深度图
add_executable(range_image_creation range_image_creation.cpp)
target_link_libraries(range_image_creation ${PCL_LIBRARIES})
target_include_directories(range_image_creation PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(range_image_creation PRIVATE ${PCL_DEFINITIONS})

add_executable(range_image_border_extraction range_image_border_extraction.cpp)
target_link_libraries(range_image_border_extraction ${PCL_LIBRARIES})
target_include_directories(range_image_border_extraction PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(range_image_border_extraction PRIVATE ${PCL_DEFINITIONS})
