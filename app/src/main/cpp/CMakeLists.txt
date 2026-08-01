cmake_minimum_required(VERSION 3.22.1)
project("latticeboltzmann")

# 1. Properly find and configure OpenMP
find_package(OpenMP REQUIRED)

add_library(latticeboltzmann SHARED lbm_engine.cpp)

target_link_libraries(
        latticeboltzmann
        PRIVATE
        OpenMP::OpenMP_CXX
        android
        log
        jnigraphics
)
