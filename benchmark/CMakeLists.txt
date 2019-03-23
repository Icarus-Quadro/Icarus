project(icarus-benchmark)

set(CMAKE_BUILD_TYPE Release)

add_executable(${PROJECT_NAME}
    benchmarkKalman.cpp
)

target_link_libraries(${PROJECT_NAME}
    icarus
    benchmark
    benchmark_main
)

# disable alignment and vectorization to better approximate performance on embedded device
target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_DONT_ALIGN=1)

# target_compile_options(${PROJECT_NAME} PUBLIC -O2)