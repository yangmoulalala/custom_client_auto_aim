cmake_minimum_required(VERSION 3.24)

project(sp_vision LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

option(SP_VISION_ENABLE_CUDA "Enable the ONNX Runtime CUDA execution path" ON)
set(ONNXRUNTIME_ROOT "/opt/onnxruntime" CACHE PATH "System ONNX Runtime installation")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

find_path(
    ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS "${ONNXRUNTIME_ROOT}/include"
    NO_DEFAULT_PATH
    NO_CACHE)
find_library(
    ONNXRUNTIME_LIBRARY onnxruntime
    HINTS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64"
    NO_DEFAULT_PATH
    NO_CACHE)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR
        "ONNX Runtime was not found under ${ONNXRUNTIME_ROOT}. "
        "Follow the installation steps in the repository README, or set ONNXRUNTIME_ROOT "
        "explicitly.")
endif()

set(ONNXRUNTIME_VERSION_FILE "${ONNXRUNTIME_ROOT}/VERSION_NUMBER")
if(NOT EXISTS "${ONNXRUNTIME_VERSION_FILE}")
    message(FATAL_ERROR
        "${ONNXRUNTIME_ROOT} has no VERSION_NUMBER file; ONNX Runtime 1.28.0 is required.")
endif()
file(STRINGS "${ONNXRUNTIME_VERSION_FILE}" ONNXRUNTIME_VERSION LIMIT_COUNT 1)
if(NOT ONNXRUNTIME_VERSION STREQUAL "1.28.0")
    message(FATAL_ERROR
        "ONNX Runtime 1.28.0 is required, but ${ONNXRUNTIME_ROOT} contains "
        "${ONNXRUNTIME_VERSION}.")
endif()

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(
    onnxruntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")

if(SP_VISION_ENABLE_CUDA)
    if(NOT EXISTS "${ONNXRUNTIME_ROOT}/lib/libonnxruntime_providers_cuda.so")
        message(FATAL_ERROR
            "${ONNXRUNTIME_ROOT} has no CUDA Execution Provider. "
            "Follow the installation steps in the repository README.")
    endif()
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
        set(CMAKE_CUDA_ARCHITECTURES 86 CACHE STRING "CUDA architectures" FORCE)
    endif()
    if(NOT CMAKE_CUDA_COMPILER AND EXISTS "/usr/local/cuda-12.8/bin/nvcc")
        set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.8/bin/nvcc"
            CACHE FILEPATH "CUDA compiler" FORCE)
    endif()
    enable_language(CUDA)
    find_package(CUDAToolkit 12.8 EXACT REQUIRED)
    find_library(
        CUDNN_LIBRARY cudnn
        HINTS /usr/lib/x86_64-linux-gnu /usr/local/cuda-12.8/lib64
        NO_CACHE)
    if(NOT CUDNN_LIBRARY)
        message(FATAL_ERROR
            "cuDNN 9 was not found. Follow the installation steps in the repository README.")
    endif()
    file(REAL_PATH "${CUDNN_LIBRARY}" CUDNN_REALPATH)
    if(NOT CUDNN_REALPATH MATCHES "libcudnn\\.so\\.9(\\.|$)")
        message(FATAL_ERROR
            "cuDNN 9 is required, but ${CUDNN_REALPATH} was found. "
            "Follow the installation steps in the repository README.")
    endif()
    message(STATUS "AUV inference backend: ONNX Runtime CUDA (GPU 0, SM 8.6)")
else()
    message(WARNING
        "Building without CUDA support. device: GPU will fail at startup; this mode is for "
        "CPU validation only.")
endif()

include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)

if(TARGET auv_io)
    add_executable(custom_client src/custom_client.cpp)
    target_link_libraries(
        custom_client PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp
        nlohmann_json::nlohmann_json auto_aim tools auv_io)

    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)

    add_executable(ros_calibrate_camera calibration/ros_calibrate_camera.cpp)
    target_link_libraries(
        ros_calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
    ament_target_dependencies(ros_calibrate_camera rclcpp sensor_msgs)

    add_executable(ros_calibrate_handeye calibration/ros_calibrate_handeye.cpp)
    target_link_libraries(
        ros_calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
    ament_target_dependencies(ros_calibrate_handeye rclcpp sensor_msgs)
endif()

if(BUILD_TESTING)
    add_executable(yolov5_model_test tests/yolov5_model_test.cpp)
    target_compile_definitions(
        yolov5_model_test PRIVATE
        CUSTOM_CLIENT_CONFIG="${PROJECT_SOURCE_DIR}/configs/custom_client.yaml"
        AUV_MODEL_PATH="${PROJECT_SOURCE_DIR}/models/0526.onnx"
        INVALID_MODEL_PATH="${PROJECT_SOURCE_DIR}/models/tiny_resnet.onnx"
        AUV_DEMO_PATH="${PROJECT_SOURCE_DIR}/assets/demo/demo.avi")
    if(SP_VISION_ENABLE_CUDA)
        target_compile_definitions(yolov5_model_test PRIVATE SP_VISION_TEST_WITH_CUDA=1)
    endif()
    target_link_libraries(
        yolov5_model_test PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools)
    add_test(NAME yolov5_model_test COMMAND yolov5_model_test)
    set_tests_properties(
        yolov5_model_test PROPERTIES TIMEOUT 90
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/..")

    add_executable(solver_image_size_test tests/solver_image_size_test.cpp)
    target_compile_definitions(
        solver_image_size_test PRIVATE
        SOLVER_IMAGE_SIZE_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/solver_image_size.yaml")
    target_link_libraries(
        solver_image_size_test PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools)
    add_test(NAME solver_image_size_test COMMAND solver_image_size_test)

    add_executable(handeye_transform_test tests/handeye_transform_test.cpp)
    target_link_libraries(handeye_transform_test PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)
    add_test(NAME handeye_transform_test COMMAND handeye_transform_test)

    add_executable(outpost_model_test tests/outpost_model_test.cpp)
    target_compile_definitions(
        outpost_model_test PRIVATE
        OUTPOST_MODEL_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/outpost_model_test.yaml")
    target_link_libraries(
        outpost_model_test PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools)
    add_test(NAME outpost_model_test COMMAND outpost_model_test)

    add_executable(aimer_spin_selection_test tests/aimer_spin_selection_test.cpp)
    target_compile_definitions(
        aimer_spin_selection_test PRIVATE
        AIMER_SPIN_SELECTION_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/aimer_spin_selection_test.yaml"
        AIMER_SPIN_SELECTION_INVALID_CONFIG="${PROJECT_SOURCE_DIR}/tests/aimer_spin_selection_invalid.yaml")
    target_link_libraries(
        aimer_spin_selection_test PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools)
    add_test(NAME aimer_spin_selection_test COMMAND aimer_spin_selection_test)

    add_executable(target_layout_test tests/target_layout_test.cpp)
    target_compile_definitions(
        target_layout_test PRIVATE
        TARGET_LAYOUT_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/target_layout_test.yaml")
    target_link_libraries(
        target_layout_test PRIVATE ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools)
    add_test(NAME target_layout_test COMMAND target_layout_test)

    if(TARGET auv_io)
        add_executable(auv_client_io_test tests/auv_client_io_test.cpp)
        target_compile_definitions(
            auv_client_io_test PRIVATE
            AUV_CLIENT_TEST_CONFIG="${PROJECT_SOURCE_DIR}/tests/auv_client_io_test.yaml")
        target_link_libraries(
            auv_client_io_test PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json auv_io)
        add_test(NAME auv_client_io_test COMMAND auv_client_io_test)
        set_tests_properties(
            auv_client_io_test PROPERTIES TIMEOUT 15
            ENVIRONMENT "ROS_LOG_DIR=/tmp;ROS_DOMAIN_ID=231")
    endif()
endif()
