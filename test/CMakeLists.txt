project(IcarusTest)

add_executable(${PROJECT_NAME}
    TestKalmanFilter.cpp
    TestMerweScaledSigmaPoints.cpp
    TestProcessModel.cpp
    TestMeasurementModel.cpp
    TestEllipsoidalCalibrator.cpp
    TestGasModel.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} icarus gmock gmock_main pthread)
target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_DONT_ALIGN=1)
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g)

add_test(${PROJECT_NAME}_ ${PROJECT_NAME})
