project(IcarusTest)

set(CMAKE_CXX_STANDARD 17)

find_package(GTest REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/Icarus)

add_executable(${PROJECT_NAME}
    testCalibrator.cpp
)

target_link_libraries(${PROJECT_NAME} Icarus ${GTEST_BOTH_LIBRARIES} pthread)
