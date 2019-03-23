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
