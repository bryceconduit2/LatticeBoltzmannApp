cmake_minimum_required(VERSION 3.22.1)
project("latticeboltzmann")

add_library(latticeboltzmann SHARED lbm_engine.cpp)

# 1. Tell the compiler to use OpenMP
target_compile_options(latticeboltzmann PRIVATE -fopenmp)

# 2. Tell the linker to use OpenMP
target_link_options(latticeboltzmann PRIVATE -fopenmp)

target_link_libraries(
        latticeboltzmann
        PRIVATE
        android
        log
        jnigraphics
)
