cmake_minimum_required(3.10)
project(Icarus)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(thirdparty)

add_subdirectory(icarus)

add_subdirectory(test)
add_subdirectory(perftest)
