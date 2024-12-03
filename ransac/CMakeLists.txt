cmake_minimum_required(VERSION 3.5)
PROJECT(moxun_ransac C CXX ASM)

file(GLOB ransac_srcs "*.cpp" "*.cc")

if (WIN32)
    add_library(moxun_ransac STATIC
            ${ransac_srcs}
    )
else ()
    add_library(moxun_ransac SHARED
            ${ransac_srcs}
    )
endif ()

target_link_libraries(moxun_ransac
        moxun_basic
        moxun_math
)

target_include_directories(moxun_ransac PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
)

if (USE_TEST)
    add_subdirectory(test)
endif ()