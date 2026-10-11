# RoboMaster 视觉与控制工程

本仓库保存当前维护的一份源码。小电脑正式目录：`/home/rm/sp_vision`。
本分支只跟踪源码、配置、构建脚本和文档；模型、SDK库、固件镜像等只在本机保留。
仓库已清除旧提交与旧标签，只保留当前最终版，不保留历史备份或带后缀的副本。

## 从这里开始

- [当前状态与未完成事项](docs/status.md)
- [完整工程架构、代码阅读、构建与调试](docs/development.md)
- [步兵固件构建、烧录与源码调试](firmware/infantry/README.md)
- [本次步兵精度改进与后续方案](步兵自瞄工作日志.md)
- [外部依赖与资源校验](docs/dependencies.md)
- [配置与标定状态](docs/configuration.md)
- [固件选择](firmware/README.md)
- [格式塔仿真](simulation/README.md)
- [后续修改规则](AGENTS.md)

## 目录

| 目录 | 用途 |
|---|---|
| `src/` | 程序入口，正式程序为 `standard` |
| `tasks/auto_aim/` | 识别、目标跟踪、预测及调度 |
| `io/` | 相机、串口及设备接口 |
| `tools/` | 公共算法、一键启动与停止脚本 |
| `calibration/` | 标定与验证工具源码 |
| `configs/standard.yaml` | 唯一正式运行配置 |
| `configs/calibration.yaml` | 标定工具配置，不能当作正式运行配置 |
| `tests/` | 离线及接口测试源码 |
| `assets/`、`librm/` | 本机模型位置（不入库）与源码依赖子模块 |
| `firmware/` | 步兵和哨兵两个硬件目标的当前固件 |
| `simulation/` | 格式塔仿真成果 |
| `docs/` | 当前使用与维护文档 |

`build/`、`logs/`、`data/` 为本机生成内容，不提交 Git。
桌面“启动自瞄”“停止自瞄”仍为现场入口，红蓝选择继续持久化，运行日志保留最近20份。

## 当前工作状态

截至2026-10-11，步兵中速预测已使用整车EKF同源几何，并包含Debug模式诊断、离线回放和预测/串口回归测试。当前正式参数为旋转水平门限0.5°、基础预测延迟85ms、配置弹速28m/s；弹速和真实离膛延迟尚未实测确认。105ms与60ms试验已结束，基础延迟恢复85ms。

小电脑现场源码、Windows仿真支持和开发/烧录说明均随源码维护。步兵与哨兵固件分别保留，仓库源码不代表车上已经烧录的版本，视觉一致性也不代表实弹命中率。模型和相机SDK需要按[依赖说明](docs/dependencies.md)及[资源清单](docs/artifacts.json)另行准备。
