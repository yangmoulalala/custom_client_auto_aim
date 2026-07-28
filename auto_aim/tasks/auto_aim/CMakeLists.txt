set(AUTO_AIM_SOURCES
    armor.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    onnx_inference.cpp)

if(SP_VISION_ENABLE_CUDA)
    list(APPEND AUTO_AIM_SOURCES cuda_preprocess.cu)
endif()

add_library(auto_aim STATIC ${AUTO_AIM_SOURCES})
target_link_libraries(
    auto_aim PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
    yaml-cpp
    tools
    onnxruntime::onnxruntime)

if(SP_VISION_ENABLE_CUDA)
    target_compile_definitions(auto_aim PRIVATE AUTO_AIM_WITH_CUDA=1)
    target_link_libraries(auto_aim PRIVATE CUDA::cudart)
    set_target_properties(
        auto_aim PROPERTIES
        CUDA_ARCHITECTURES 86
        CUDA_STANDARD 17
        CUDA_STANDARD_REQUIRED ON)
endif()
