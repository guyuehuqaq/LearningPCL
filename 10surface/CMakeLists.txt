cmake_minimum_required(VERSION 3.10)
project(Surface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL CONFIG REQUIRED)

add_executable(resample resample.cpp)
target_link_libraries(resample PRIVATE ${PCL_LIBRARIES})

add_executable(hull_2d hull_2d.cpp)
target_link_libraries(hull_2d PRIVATE ${PCL_LIBRARIES})

add_executable(greedy_projection greedy_projection.cpp)
target_link_libraries(greedy_projection PRIVATE ${PCL_LIBRARIES})

add_executable(bspline_fitting bspline_fitting.cpp)
target_link_libraries(bspline_fitting PRIVATE ${PCL_LIBRARIES})

add_executable(fitting_curve_2d fitting_curve_2d.cpp)
target_link_libraries(fitting_curve_2d PRIVATE ${PCL_LIBRARIES})