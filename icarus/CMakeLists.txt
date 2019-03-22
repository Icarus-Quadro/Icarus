project(icarus)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} STATIC
    "source/Aircraft.cpp"
    "source/ElectricMotor.cpp"
    "source/HoverNavigator.cpp"
    "source/PIDController.cpp"
    "source/EllipsoidalCalibrator.cpp"
    "source/EllipsoidalCalibration.cpp"
)

target_include_directories(${PROJECT_NAME}
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/icarus"
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES=1)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
