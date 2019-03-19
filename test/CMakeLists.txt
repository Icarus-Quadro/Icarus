project(IcarusTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(googletest)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/Icarus)

add_executable(${PROJECT_NAME}
    testKalmanFilter.cpp
    testMerweScaledSigmaPoints.cpp
    testRigidBodyProcessModel.cpp
)

target_link_libraries(${PROJECT_NAME} Icarus gmock gmock_main pthread)
target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_DONT_ALIGN=1)
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g)