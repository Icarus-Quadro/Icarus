project(IcarusTest)

add_executable(${PROJECT_NAME}
    testKalmanFilter.cpp
    testMerweScaledSigmaPoints.cpp
    testRigidBodyProcessModel.cpp
)

target_link_libraries(${PROJECT_NAME} icarus gmock gmock_main pthread)
target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_DONT_ALIGN=1)
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g)
