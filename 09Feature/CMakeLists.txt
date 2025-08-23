cmake_minimum_required(VERSION 3.10)
project(Feature)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED)
add_executable(normal_estimation normal_estimation.cpp)
target_link_libraries(normal_estimation ${PCL_LIBRARIES})

add_executable(normal_estimation_subpoints normal_estimation_subpoints.cpp)
target_link_libraries(normal_estimation_subpoints ${PCL_LIBRARIES})

add_executable(normal_estimation_intergral_images normal_estimation_using_integral_images.cpp)
target_link_libraries(normal_estimation_intergral_images ${PCL_LIBRARIES})

add_executable(PFH PFH.cpp)
target_link_libraries(PFH ${PCL_LIBRARIES})

add_executable(FPFH FPFH.cpp)
target_link_libraries(FPFH ${PCL_LIBRARIES})

add_executable(VFH VFH.cpp)
target_link_libraries(VFH ${PCL_LIBRARIES})

add_executable(NARF NARF.cpp)
target_link_libraries(NARF ${PCL_LIBRARIES})



