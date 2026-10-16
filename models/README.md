# 当前使用的模型

2026-10-17按用户选择，仅保存小电脑当前使用的三个模型文件，合计约4.7 MB。其他模型和所有日志均不上传。来源、字节数及SHA-256见[清单](manifest.json)。

| 分类 | 文件 | 用途 |
|---|---|---|
| detection/openvino/yolo26/ | yolo26.xml、yolo26.bin | 当前YOLO26装甲板检测器，XML/BIN配对保存 |
| classification/ | tiny_resnet.onnx | 当前数字分类器 |

小电脑运行资源继续位于assets/，正式YAML路径未修改。本目录是Git分支中的模型快照，不生成安装器或恢复包；SDK动态库、固件镜像、可执行文件、日志和演示视频均未包含。
