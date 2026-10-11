cmake_minimum_required(VERSION 3.16)

# 基础源文件（始终编译）
set(AUTO_AIM_SOURCES
    adaptive_corner_refiner.cpp
    armor.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
)

# Linux 专属源文件：commandgener 调用 io::CBoard（硬件 CAN）并引入
# plotter.hpp -> netinet/in.h POSIX 头链；planner/tinympc 依赖 dirent.h。
# Windows gestalt 仿真不使用（与 bridge 版 Windows 构建一致）。
if(NOT WIN32)
    list(APPEND AUTO_AIM_SOURCES
        multithread/commandgener.cpp
        planner/planner.cpp
    )
endif()

# 根据可用后端添加YOLO实现源文件
set(YOLO_SOURCES "")

if(OpenVINO_FOUND)
    message(STATUS "[auto_aim] Adding OpenVINO YOLO implementations")
    list(APPEND YOLO_SOURCES
        classifier.cpp
        yolos/yolov5.cpp
        yolos/yolov8.cpp
        yolos/yolo11.cpp
        yolos/yolo26.cpp
    )
    if(NOT WIN32)
        # mt_detector 依赖 io::CBoard，仅 Linux/真机路径编译
        list(APPEND YOLO_SOURCES multithread/mt_detector.cpp)
    endif()
endif()

if(TensorRT_FOUND)
    message(STATUS "[auto_aim] Adding TensorRT YOLO implementations")

    # CUDA 语言已在主 CMakeLists.txt 中启用，这里只需添加源文件
    if(CUDA_NVCC_EXECUTABLE)
        list(APPEND YOLO_SOURCES
            yolos/tensorrt_logger.cpp
            yolos/yolov8_trt.cpp
            yolos/yolo11_trt.cpp
            yolos/cuda_preprocess.cu
        )
    else()
        message(WARNING "[auto_aim] TensorRT found but CUDA compiler (nvcc) not available. TensorRT support will be disabled.")
        set(TensorRT_FOUND FALSE)
    endif()
endif()

# 合并所有源文件
list(APPEND AUTO_AIM_SOURCES ${YOLO_SOURCES})

# 检查 ROS2 环境（用于 tracker 的 marker 功能）
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(visualization_msgs QUIET)

if(WIN32)
    # Windows gestalt 分支：tinympc 不构建，提供空 INTERFACE 目标保持链接一致
    add_library(tinympcstatic INTERFACE)
else()
    add_subdirectory(planner/tinympc)
endif()

add_library(auto_aim OBJECT ${AUTO_AIM_SOURCES})

# 基础链接（Windows gestalt 分支不构建 io 硬件库，auto_aim 只用到 io 的头文件）
if(NOT WIN32)
    target_link_libraries(auto_aim PUBLIC io)
endif()
target_link_libraries(auto_aim PUBLIC tinympcstatic)

# include 传播：Windows/vcpkg 不提供系统 include 路径，须经 target 暴露
# fmt/spdlog/yaml-cpp 头文件（Linux 系统包在默认路径，行为不变）
target_link_libraries(auto_aim PUBLIC fmt::fmt spdlog::spdlog yaml-cpp)
if(WIN32)
    # MSVC 默认不定义 M_PI，yaw_eskf/measurement_noise 等头文件依赖它
    target_compile_definitions(auto_aim PRIVATE _USE_MATH_DEFINES)
    # Windows 兼容 shim：sched.h（CPU 亲和性 no-op）、serial 纯头 include 路径、
    # fmt 枚举 formatter（/FI 强制预包含，见 thirdparty/win_shims/）。
    # PUBLIC：gestalt 编译头文件链（aimer.hpp -> cboard.hpp -> serial.h）也需要。
    target_include_directories(auto_aim PUBLIC
        ${PROJECT_SOURCE_DIR}/thirdparty/win_shims
        ${PROJECT_SOURCE_DIR}/io/serial/include)
    target_compile_options(auto_aim PRIVATE
        /FI"${PROJECT_SOURCE_DIR}/thirdparty/win_shims/win_fmt_enum.hpp")
endif()

# 条件链接：OpenVINO
if(OpenVINO_FOUND)
    target_compile_definitions(auto_aim PUBLIC ENABLE_OPENVINO)
    target_link_libraries(auto_aim PUBLIC openvino::runtime)
    message(STATUS "[auto_aim] OpenVINO support enabled")
endif()

# 条件链接：TensorRT
if(TensorRT_FOUND)
    target_compile_definitions(auto_aim PUBLIC ENABLE_TENSORRT)

    # 添加 TensorRT include 目录
    target_include_directories(auto_aim PUBLIC ${TensorRT_INCLUDE_DIR})

    # 添加 CUDA include 目录（从 CMAKE_CUDA_COMPILER 推导）
    if(CMAKE_CUDA_COMPILER)
        get_filename_component(CUDA_BIN_DIR "${CMAKE_CUDA_COMPILER}" DIRECTORY)
        get_filename_component(CUDA_ROOT_DIR "${CUDA_BIN_DIR}" DIRECTORY)
        target_include_directories(auto_aim PUBLIC "${CUDA_ROOT_DIR}/include")
        message(STATUS "[auto_aim] CUDA include directory: ${CUDA_ROOT_DIR}/include")
    endif()

    # 链接 TensorRT 库
    target_link_libraries(auto_aim PUBLIC ${TensorRT_LIBRARIES})

    # 设置 CUDA 架构（继承主 CMakeLists 的设置）
    if(CMAKE_CUDA_ARCHITECTURES)
        set_property(TARGET auto_aim PROPERTY CUDA_ARCHITECTURES ${CMAKE_CUDA_ARCHITECTURES})
    endif()

    message(STATUS "[auto_aim] TensorRT support enabled with CUDA preprocessing")
endif()

# 如果 ROS2 可用，添加 ROS2 支持
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND visualization_msgs_FOUND)
    message(STATUS "[auto_aim] ROS2 support enabled for tracker marker visualization")
    target_compile_definitions(auto_aim PRIVATE AMENT_CMAKE_FOUND)
    target_link_libraries(auto_aim PUBLIC rclcpp::rclcpp)
    ament_target_dependencies(auto_aim rclcpp std_msgs visualization_msgs)
else()
    message(STATUS "[auto_aim] ROS2 not available, tracker marker visualization disabled")
endif()
