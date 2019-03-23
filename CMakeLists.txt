cmake_minimum_required(VERSION 3.5)
project(Icarus)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(thirdparty)

add_subdirectory(icarus)

add_subdirectory(test)
add_subdirectory(benchmark)
