# 构建与运行依赖

本仓库只跟踪源码、配置、构建脚本和文档。模型权重、SDK共享库、固件镜像、录像和运行日志不进入当前 Git 文件树。

## 源码依赖

`librm` 为 Git 子模块，固定提交 `52ec0174ee394d1fad06254b8947e8d60ba591d3`，含 etl 与 kalman 子模块。
使用 `git clone --recurse-submodules` 或 `git submodule update --init --recursive` 获取。其许可证随上游源码保留。
系统依赖沿用 ROS 2 Humble、OpenVINO 2024.6、OpenCV、Eigen、Ceres、fmt、spdlog、yaml-cpp。
固件需要独立安装 GNU Arm 工具链和 STM32CubeF4 SDK，见 `firmware/README.md`。

## 本机二进制依赖

相机 SDK 的头文件在 `io/`，共享库由厂商 SDK 提供。当前构建使用：

- `io/hikrobot/lib/amd64/libMvCameraControl.so`（ARM 为 `arm64`）
- `io/mindvision/lib/amd64/libMVSDK.so`（ARM 为 `arm64`）

当前正式模型文件为 `assets/yolo26.xml`、`assets/yolo26.bin`、`assets/tiny_resnet.onnx`。
这些文件仍保留在现有机器，只取消 Git 跟踪。当前运行资源的路径和哈希见 `artifacts.json`。
模型训练源码和可复现训练数据目前未提供，所以仅凭本仓库不能从头训练出相同权重；不得声称整个系统已完全可复现构建。

## 新克隆的资源准备

用户已明确允许为2026-10-11现场版本创建Release和发布标签。Git文件树仍不存放二进制资源；`field-2026.10.11` Release的`runtime.tar.gz`保存当前模型、两种架构的清单内SDK库、现场x86_64程序、MVS、OpenVINO 2024.6、ROS 2 Humble及该程序实际链接的非glibc系统库。厂商随附许可证随资源保留。
在Ubuntu 22.04 x86_64上，按[快速恢复](recovery.md)下载并校验该附件；资源逐文件SHA-256与整个压缩包SHA-256均有检查。Ubuntu本身、内核、glibc、图形会话、USB设备和下位机固件不由该附件替代。源码依赖与编译环境版本见`runtime-environment.json`；只有需要重新编译时才必须准备编译依赖和递归子模块。
运行 `python3 scripts/check_runtime_assets.py` 校验当前模型和 SDK。该脚本只读取文件，不联网、不恢复旧文件、不启动程序。
现有小电脑与Windows工作区的当前运行资源原地保留。仅执行`git clone`仍不会自动下载Release附件；恢复工具负责下载、校验和安装，不应跳过这一步。
