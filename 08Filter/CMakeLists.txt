cmake_minimum_required(VERSION 3.10)
project(Filter)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED)

add_executable(pass_through_filter pass_through_filter.cpp)
target_link_libraries(pass_through_filter ${PCL_LIBRARIES})

add_executable(voxel_grid_downsample voxel_grid.cpp)
target_link_libraries(voxel_grid_downsample ${PCL_LIBRARIES})

add_executable(statistical_removal statistical_removal.cpp)
target_link_libraries(statistical_removal ${PCL_LIBRARIES})

add_executable(project_inliers project_inliers.cpp)
target_link_libraries(project_inliers ${PCL_LIBRARIES})

add_executable(extract_indices extract_indices.cpp)
target_link_libraries(extract_indices ${PCL_LIBRARIES})

add_executable(remove_outliers remove_outliers.cpp)
target_link_libraries(remove_outliers ${PCL_LIBRARIES})

add_executable(bilateral_filter bilateral_filter.cpp)
target_link_libraries(bilateral_filter ${PCL_LIBRARIES})