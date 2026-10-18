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

新环境需单独安装ROS 2 Humble、OpenVINO 2024.6和厂商相机SDK，并向资源持有人取得上述模型文件。源码依赖通过固定的Git子模块获取；模型训练源码和训练数据不在本分支中。

Ubuntu常用编译依赖：

```bash
sudo apt-get install build-essential cmake libopencv-dev libeigen3-dev \
  libceres-dev libfmt-dev libspdlog-dev libyaml-cpp-dev nlohmann-json3-dev \
  libusb-1.0-0-dev python3-yaml
git submodule update --init --recursive
```

运行 `python3 scripts/check_runtime_assets.py` 校验当前模型和 SDK。该脚本只读取文件，不联网、不恢复旧文件、不启动程序。
资源就绪后使用 `bash scripts/build.sh standard` 编译，产物只写入 `build/`。现有小电脑和Windows工作区的运行资源原地保留，本次没有重新安装环境或替换资源。
