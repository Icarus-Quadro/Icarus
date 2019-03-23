cmake_minimum_required(VERSION 3.5)
project(Icarus)

set(CMAKE_CXX_STANDARD 17)

# disable alignment and vectorization to better approximate performance on embedded device
add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=0 EIGEN_DONT_VECTORIZE=1 EIGEN_UNALIGNED_VECTORIZE=0)

add_subdirectory(thirdparty)

add_subdirectory(icarus)

add_subdirectory(test)
add_subdirectory(benchmark)
