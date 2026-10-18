cmake_minimum_required(VERSION 3.16)

# 基础源文件列表
set(TOOLS_SOURCES
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
    float16.cpp
)

if(NOT WIN32)
    # plotter 是 POSIX-UDP（PlotJuggler 用），Windows gestalt 构建排除；
    # gestalt 驱动不引用 Plotter（与 bridge 版 Windows 构建一致）。
    list(APPEND TOOLS_SOURCES plotter.cpp)
endif()

# 检查ROS2包是否可用（在父CMakeLists.txt中已经find_package）
if(rclcpp_FOUND AND tf2_ros_FOUND AND geometry_msgs_FOUND AND visualization_msgs_FOUND)
    list(APPEND TOOLS_SOURCES ros2_visualizer.cpp)
    message(STATUS "[Tools] ROS2 Visualizer enabled")
endif()

add_library(tools STATIC ${TOOLS_SOURCES})

if(WIN32)
    # MSVC/vcpkg 无系统 include 路径，经 target 传播；/FInumeric 补齐
    # libstdc++ 会传递式引入而 MSVC STL 不会的 <numeric>
    target_compile_options(tools PRIVATE /utf-8 /FInumeric)
    target_link_libraries(tools PUBLIC spdlog::spdlog fmt::fmt ${OpenCV_LIBS})
endif()

# 如果ROS2可用，先设置ROS2依赖（ament_target_dependencies必须在其他链接之前）
if(rclcpp_FOUND AND tf2_ros_FOUND AND geometry_msgs_FOUND AND visualization_msgs_FOUND)
    # 定义AMENT_CMAKE_FOUND宏，让ros2_visualizer.cpp中的代码被编译
    target_compile_definitions(tools PUBLIC AMENT_CMAKE_FOUND)

    ament_target_dependencies(tools PUBLIC
        rclcpp
        tf2_ros
        geometry_msgs
        visualization_msgs
    )
endif()

# 链接fmt库以解决undefined reference错误
target_link_libraries(tools PUBLIC fmt::fmt)