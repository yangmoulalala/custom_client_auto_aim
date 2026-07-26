cmake_minimum_required(VERSION 3.16)
project(custom_client_adapter LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(JPEG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)

add_executable(rm_video_node
  src/compressed_image_publisher.cpp
  src/rm_video_node.cpp
  src/video_decoder.cpp
)
target_include_directories(rm_video_node PRIVATE include)
target_link_libraries(rm_video_node JPEG::JPEG PkgConfig::FFMPEG)
ament_target_dependencies(rm_video_node rclcpp sensor_msgs std_msgs)
target_compile_options(rm_video_node PRIVATE -Wall -Wextra -Wpedantic)

ament_python_install_package(rm_mqtt)

install(TARGETS rm_video_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/rm_mqtt_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch proto
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(rm_mqtt_tests test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
