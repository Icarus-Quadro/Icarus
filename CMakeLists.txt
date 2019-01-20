project(Icarus)

add_library(${PROJECT_NAME} STATIC
    "source/Aircraft.cpp"
    "source/ElectricMotor.cpp"
    "source/HoverNavigator.cpp"
    "source/KalmanFilter.cpp"
    "source/PIDController.cpp"
)

target_include_directories(${PROJECT_NAME}
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/icarus"
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES=1)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
