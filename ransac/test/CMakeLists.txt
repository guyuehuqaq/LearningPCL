cmake_minimum_required(VERSION 3.5)

add_executable(combination_sampler_test combination_sampler_test.cc)
target_link_libraries(combination_sampler_test moxun_ransac gtest gtest_main)

add_executable(progressive_sampler_test progressive_sampler_test.cc)
target_link_libraries(progressive_sampler_test moxun_ransac gtest gtest_main)

add_executable(random_sampler_test random_sampler_test.cc)
target_link_libraries(random_sampler_test moxun_ransac gtest gtest_main)

add_executable(support_measurement_test support_measurement_test.cc)
target_link_libraries(support_measurement_test moxun_ransac gtest gtest_main)