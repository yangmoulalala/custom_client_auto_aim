find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(sensor_msgs QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND sensor_msgs_FOUND)
    message(STATUS "ROS2 standard messages found, compiling custom client I/O.")
    add_library(auv_io STATIC auv_client.cpp)
    target_link_libraries(
        auv_io PUBLIC yaml-cpp nlohmann_json::nlohmann_json ${OpenCV_LIBS} Eigen3::Eigen)
    ament_target_dependencies(auv_io PUBLIC rclcpp sensor_msgs std_msgs)
else()
    message(STATUS "ROS2 standard message environment not found; skipping custom client and ROS calibration targets.")
endif()
