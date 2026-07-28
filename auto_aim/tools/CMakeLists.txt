cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT 
    extended_kalman_filter.cpp
    img_tools.cpp
    math_tools.cpp
    trajectory.cpp
    logger.cpp
)

target_link_libraries(tools PUBLIC ${OpenCV_LIBS} Eigen3::Eigen fmt::fmt spdlog::spdlog)
