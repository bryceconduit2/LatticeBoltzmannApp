cmake_minimum_required(VERSION 3.22.1)
project("fluidsandbox")

# 1. Properly find and configure OpenMP
find_package(OpenMP REQUIRED)

add_library(fluidsandbox SHARED lbm_engine.cpp)

target_link_libraries(
        fluidsandbox
        PRIVATE
        OpenMP::OpenMP_CXX
        android
        log
        jnigraphics
)
