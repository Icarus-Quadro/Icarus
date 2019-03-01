project(IcarusTest)

set(CMAKE_CXX_STANDARD 17)

find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}
    testCalibrator.cpp
)

target_link_libraries(${PROJECT_NAME} ${GTEST_BOTH_LIBRARIES} pthread)
