# 工程架构与开发调试指南

核对日期：2026-10-11，包含当日下午的参数试验。今日证据和后续方案见[工作日志](../步兵自瞄工作日志.md)，主控操作见[步兵构建、烧录与调试](../firmware/infantry/README.md)。

## 1. 开发与运行位置

| 部分 | 正式位置 | 用途 |
|---|---|---|
| Windows开发机 | `D:\git_clone\sp-vision-2026-integration-archive` | 编辑、分析；当前本地build缓存用于Windows格式塔仿真 |
| 小电脑Linux | `/home/rm/sp_vision` | 运行`build/standard`，相机、识别、跟踪、预测、串口控制 |
| 步兵STM32F405 | 源码`firmware/infantry/` | FreeRTOS、遥控、IMU、CAN、云台与供弹；需独立编译和烧录 |
| 哨兵STM32F405 | 源码`firmware/sentry/` | 另一硬件与协议，不能混刷 |

上位机C++改动在小电脑编译，由用户重启程序后生效；只有STM32源码改动需要烧录。YAML改动通常重启加载。`standard_mpc`、其他debug入口、Windows `gestalt`不代表当前现场路径。

仅维护现有文件；不创建源码/配置/工程副本、备份分支或标签。`linux步兵/`是以前的研究目录，不是当前开发与同步入口。现有本机仿真改动与小电脑构建文件有差异，不能整仓覆盖。

正式运行配置只有`configs/standard.yaml`；标定用`configs/calibration.yaml`。启动器生成`logs/xuc-bringup/field_autoaim.runtime.yaml`，只覆盖本次敌方颜色，是缓存。颜色偏好在`~/.config/sp-vision/field_autoaim_enemy_color`。

## 2. 从图像到电机的整体数据流

~~~mermaid
flowchart TD
  CAM[相机] --> CAP[io/camera与hikrobot<br/>图像和主机时间戳]
  RX[下位机反馈] --> XUC[io/xuc接收线程<br/>IMU历史]
  XUC --> ALIGN[standard配对帧时刻姿态]
  CAP --> DET[YOLO26<br/>角点、颜色、类别、置信度]
  ALIGN --> SOLVE[Solver<br/>PnP和坐标变换]
  DET --> TRACK[Tracker<br/>身份与装甲面关联]
  SOLVE <--> TRACK
  TRACK --> TARGET[Target<br/>整车EKF、Yaw ESKF、单板运动]
  TARGET --> MODE[RotationModelManager]
  TARGET --> AIM[Aimer与Trajectory<br/>未来预测、选板、飞行时间]
  MODE --> STD[standard整合<br/>中心跟随、直接测量、限速、开火条件]
  AIM --> STD
  PHASE[高速相位事件调度] --> STD
  STD --> SEND[XucSender<br/>高低电平周期、角度保持]
  SEND --> MCU[STM32<br/>运动和供弹状态机]
  MCU --> RX
  STD -. Debug快照 .-> LOG[有界异步日志]
~~~

`src/standard.cpp`是实际整合入口。Aimer返回的yaw/pitch随后还会经过中心跟随/直接测量、修正、限速和门控；XucSender在高电平期间会保持接受请求时的角度。因此不能只读Aimer就判断真实云台命令。

`control`是控制许可，`shoot`是软件请求。软件请求、串口写成功、拨弹动作、真实出弹、实际命中必须分别记录。

## 3. 文件地图与修改入口

路径均相对根目录；先读hpp接口，再看cpp实现和standard调用点。

| 文件/目录 | 职责、关键接口 | 对应问题 |
|---|---|---|
| `src/standard.cpp` | `main`、初始化、每帧循环、模式、最终指令/门控、快照 | 实际行为整合 |
| `io/camera.*`、`io/hikrobot/hikrobot.*` | `read`、SDK、时间戳与队列 | 采图、时龄、积压 |
| `tasks/auto_aim/yolo.*`、`yolos/yolo26.*` | `detect`、当前模型推理、颜色/角点处理 | 检测；本次精度主线暂不改 |
| `tasks/auto_aim/armor.hpp` | `Armor`板型、角点、各坐标系观测 | 输入数据结构 |
| `tasks/auto_aim/solver.*` | `solve`、`set_R_gimbal2world`、重投影、yaw优化 | PnP、标定和坐标 |
| `tasks/auto_aim/tracker.*` | `track`、初始化/丢失/重建、关联、主观测 | 身份与切板 |
| `tasks/auto_aim/target.*` | `predict/update/update_batch`、几何重建 | 运动/观测模型 |
| `tools/extended_kalman_filter.*` | 通用EKF预测与校正 | 数值滤波实现 |
| `tasks/auto_aim/yaw_eskf.hpp` | SO(2)相位、角速度、加速度、可信度 | 旋转动态；不是完整SO(3)导航ESKF |
| `tasks/auto_aim/armor_motion_filter.hpp` | 单块装甲板运动 | 单板路径 |
| `tasks/auto_aim/rotation_model_manager.hpp` | `armor/vehicle/high_speed_phase`及滞回/信任 | 模式过渡 |
| `tasks/auto_aim/aimer.*` | `aim`、`choose_aim_point` | 预测时刻、选板、弹道迭代 |
| `tools/trajectory.*` | `Trajectory(v0,d,h)` | 重力抛体短飞行时间解；当前无空气阻力 |
| `tasks/auto_aim/shooter.*` | 原生开火判断 | 还须看standard的后续覆盖 |
| `tasks/auto_aim/gyro_phase_scheduler.hpp` | 高速相位事件、命令到期时刻 | 高速时机 |
| `tasks/auto_aim/gyro_face_selector.hpp` | 同一高速事件的板ID承诺 | 高速选板 |
| `tasks/auto_aim/robust_range_filter.hpp` | 模型/测量稳健距离 | 直接俯仰与事件飞行时间 |
| `tasks/auto_aim/shot_delay_adapter.hpp` | 基于反馈的延迟修正 | 当前配置关闭；动作反馈不等于离膛 |
| `io/command.hpp` | `Command{control,shoot,yaw,pitch}` | 内部控制载体 |
| `io/xuc.*` | `XucSender`、收包、IMU历史、发送保持、异步诊断 | 协议和实际发送 |
| `tools/logger.*`、`tools/ros2_visualizer.*` | 日志/显示 | 观察，不是真值 |
| `tools/field_start_autoaim.sh`、`field_stop_autoaim.sh` | 桌面入口、环境、颜色、配置缓存 | 用户自行启动/停止 |
| `scripts/analyze_prediction_debug.py` | 同epoch/face未来匹配、归因、PnP离线复算 | 先分析后改生产 |
| `tests/` | 离线夹具及硬件测试混存 | 运行前确认是否访问硬件 |
| `calibration/` | 内参、手眼、验证 | 标定工具 |
| `tasks/auto_buff/`、`tasks/omniperception/`、`tasks/auto_aim/planner/` | 能量机关、全向感知和规划等 | 其他功能，不是当前默认修改入口 |
| `simulation/`、`src/gestalt.cpp`、`io/gestalt/` | Windows仿真 | 不代替实车执行验收 |
| `assets/`、`librm/`、`thirdparty/` | 模型位置、子模块及依赖 | 见依赖清单，二进制/模型不入Git |

## 4. 状态、坐标与时间

### 4.1 三种状态接口

整车EKF的11维状态：

~~~text
[cx, vx, cy, vy, cz, vz, body_yaw, omega, r1, r2_delta, height_delta]
~~~

单位分别是m、m/s、rad、rad/s；第二半径为`r1+r2_delta`。`primary_face_id`是当前主观测板，`native_face_id`是未来瞄准板，二者可以不同。

- `target.ekf().x`：原始整车状态，几何变量同源。
- `target.ekf_x()`：原有组合状态；ESKF可信时替换相位/角速度，中心/速度/半径来自整车EKF。
- `whole_ekf_armor_xyza_list()`：本次新增同源几何入口；原`armor_xyza_list()`与其他调用者保留。

仅在可信新鲜vehicle、四板非前哨站、非静止且几何有效时，Aimer选择`AimGeometry::whole_ekf`，本帧所有弹道迭代固定来源。ESKF仍参与可信度、模式和高速路径。ESKF瞬时速度低于`decision_speed=0.8`时仍可能回退existing，需另作评估。

### 4.2 坐标

Solver先求相机坐标，再执行：

~~~text
p_gimbal = R_camera2gimbal * p_camera + t_camera2gimbal
p_world  = R_gimbal2world * p_gimbal
~~~

`R_gimbal2world`来自配对IMU和`R_gimbal2imubody`；world是工程IMU参考坐标，不是独立场地定位。`yaw=atan2(y,x)`，pitch在Aimer输出时有符号转换，直接俯仰路径还会覆盖。必须结合`xuc_yaw_sign`、`xuc_imu_yaw_sign`和零位修正阅读，不凭字段名推断方向/单位。

### 4.3 预测时间

当前`aim(...,to_now=true)`近似执行：

~~~text
t_base   = t_Aimer当前时刻 + 配置有效延迟
t_impact = t_base + 迭代飞行时间
从图像时刻外推跨度 = 已经过的处理时间 + 有效延迟 + 飞行时间
~~~

处理时龄已经计入，不能再把约30ms重复加到85ms。85ms是配置有效延迟，未证明等于真实离膛延迟。

Aimer按值接收Target、用副本迭代，未来预测不会写回Tracker当前状态。主滤波仍可能存在偏差，但没有“从第三发开始把未来预测逐发累计回去”的机制。

`frame_ns`、IMU及发送时间使用主机steady-clock纳秒；日志墙钟用于人工对齐，不跨进程混减不同时间基准。图像时间戳是主机接收时刻加配置偏移，并非已测得曝光中心；IMU对齐15ms也不是完整物理同步证明。

## 5. 开火链和协议

| 层次 | 当前行为 |
|---|---|
| armor/静止 | 测量水平3°门限、120ms稳定时间等 |
| vehicle/中速 | 未来水平0.5°门限、连续2个合格帧；没有保证误差恰好为0 |
| high_speed_phase/高速 | 相位事件、时间窗口、未来板及信任条件 |
| 最终输出 | 中心跟随/直接测量、−1.3°水平零位、最终yaw限速30°/s等 |
| XucSender | 请求被接受后shoot=1约450ms，并保持该次yaw/pitch；首次成功shoot=0后低550ms，再重新判定 |
| 实际主控 | 按车上固件执行，当前版本未知；一请求是否一发须验证 |

450ms是信号保持长度，不是“等450ms才发”，也不是实测延迟。控制失效可能提前结束高电平。450+550ms不能直接叫固定1Hz或真实发弹间隔。源码还有450/200ms等旧注释，当前正式配置是450/550ms。

USART6、460800、SP包头、CRC16，角度rad：

| 方向 | 长度 | 字段顺序 |
|---|---:|---|
| 上→下 | 14B | head[2], control(u8), shoot(u8), yaw(float32), pitch(float32), crc16 |
| 下→上 | 20B | head[2], mode(u8), robot_id(u8), bullet_speed(float32), bullet_count(u16), imu_pitch(float32), imu_yaw(float32), crc16 |

`io/xuc.hpp`的Rx/Tx命名沿用下位机视角，优先确认方向。当前仓库固件复用bullet_speed为拨弹RPM、bullet_count为摩擦轮RPM打包、robot_id为门控位；车上不一定相同，不能按名字解作弹速/发数。

速度链仍有潜在不一致：standard接受反馈5–40，Aimer接受5–35、无效回退23；直接俯仰和事件用配置28。新查四轮请求的反馈字段均为0，全部选用配置28，未见这些请求间来源切换；28仍不是实测速度。统一入口前需确认字段语义与新鲜度。

## 6. Debug与离线分析

`infantry_diagnostics_enabled`控制新增诊断。简洁模式须在最外层关闭采集、构造、分配、格式化、异步写入及影子预测；仅调低日志级别仍可能执行参数求值。正常算法所需计算继续执行。

`tools/toggle_infantry_diagnostics.sh`修改正式配置，用户重启生效。本轮未执行。运行Debug开关与编译Debug不同，Release程序也能记录诊断。

程序日志`logs/YYYY-MM-DD_HH-MM-SS.log`；启动器记录`logs/xuc-bringup/日期/`；原始图像等在`data/`。启动器只保留最近20份一键日志，不要误认所有证据永远保留。

`[SHOT_DEBUG]`以`(run,id)`关联request_snapshot、first_high、first_low；prediction_frame含frame_seq/track_epoch/未来板。请求编号不是实际弹丸编号。`tx_write_complete_ns`仅表示主机写操作完成，不是串口线全部发完、下位机收包或离膛。

按改动范围在小电脑执行已审阅的离线测试：

~~~bash
cd /home/rm/sp_vision
python3 tests/test_prediction_debug.py
python3 tests/test_aimer_prediction_trace.py
python3 tests/test_xuc_shot_diagnostics.py
~~~

这些夹具不访问现场串口。此前小电脑分析测试24项通过，本机4项因缺OpenCV跳过且小电脑覆盖；Aimer真实C++测试验证几何/选板/回退和Debug输出一致，假串口验证命令周期及诊断隔离。本轮文档更新不重复算法测试。其他`camera_*`、`xuc_link_test`不能因名含test就当离线程序。

只读分析：

~~~bash
python3 scripts/analyze_prediction_debug.py logs/2026-10-10_23-19-07.log --start "2026-10-10 23:19:09"
~~~

可加`--pnp-replay`；跨午夜用完整日期。不加`--output`只打印，避免覆盖当前多轮汇总`logs/prediction_accuracy.json`。该工具的输出参数会覆盖文件，不能把单轮输出直接写到已有多轮报告而丢掉证据。

未来误差按同epoch/同一物理板/指定未来时刻的后续PnP匹配，衡量视觉一致性，不是弹着真值或命中率。

## 7. 上位机编译、同步与调试

依赖沿用现场ROS 2 Humble、OpenVINO 2024.6、OpenCV、Eigen、Ceres、fmt、spdlog、yaml-cpp和MVS，见[依赖说明](dependencies.md)。新克隆需准备资源与`librm`子模块：

~~~bash
git submodule update --init --recursive
~~~

小电脑正式编译：

~~~bash
cd /home/rm/sp_vision
bash scripts/build.sh standard
~~~

脚本source ROS环境后使用唯一build，默认并行2。编译不会替换已运行进程，由用户停止并重启；桌面入口对应field_start/field_stop脚本。助手不得自行启动/停止或用启动器--check当只读检查。

Windows连接：

~~~powershell
ssh -o BatchMode=yes -o ConnectTimeout=8 -o HostName=192.168.50.2 mini
~~~

可经已授权SSH直接编辑小电脑，或本机修改后逐文件同步。先比较对端变化和哈希，不能覆盖独立工作。仅当这个文件确实是本轮审阅修改时，单文件同步示例如下：

~~~powershell
scp -o HostName=192.168.50.2 tasks/auto_aim/aimer.cpp mini:/home/rm/sp_vision/tasks/auto_aim/aimer.cpp
~~~

头文件/调用点也要一并核对再构建。本机build是Windows仿真缓存，不能用于Linux standard，不把本地CMakeLists直接覆盖现场。

上位机优先用日志与离线夹具。当前根CMakeLists直接设Release，仅传`-DCMAKE_BUILD_TYPE=Debug`未必生效；需要符号时先审阅构建规则，在现有build构建，结束后恢复正式构建验证。现场断点暂停控制循环，会改变时序，不适合测发弹延迟。

## 8. 自己写代码的阅读顺序

1. 读command.hpp、armor.hpp、aimer.hpp、target.hpp，理解输入输出与11维状态。
2. 顺standard读`camera.read → detector.detect → tracker.track → aimer.aim → 最终门控 → xuc.send`，标出Aimer结果被覆盖的位置。
3. 选一条request_snapshot，以run/id找到first_high/first_low，再看对应prediction_frame，核对板ID、弹速、时间和最终yaw。
4. 第一项练习建议扩展现有离线分析：每次中速请求处于窗口进入/中心附近/离开哪侧，各门何时允许、同板误差趋势怎样。局部差分只作线索，不能直接认定真实发晚。
5. 分别验证固定水平偏置、有效时间偏差、逐发执行抖动，证据指向某处后再经用户确认修改一项行为。

| 想改什么 | 首先读哪里 | 验证什么 |
|---|---|---|
| 中速时机 | standard的vehicle分支、Aimer时间、XucSender | 同板窗口、重装填、两旋向、真实命中与实际发数 |
| 预测位置 | Target/Aimer/Solver | 当前残差与未来误差分别评价 |
| 切板抖动 | Tracker关联、primary_face、模式管理 | epoch/板ID不混算 |
| 云台跟不动 | 最终限速、保持、IMU、实车固件 | 原始预测/最终命令/反馈逐层对照 |
| 拨弹迟缓 | 主控xuc_aim与SHOOTER::Update | 请求、动作、出弹分开 |
| 编译没生效 | 文件哈希、进程启动时间、路径 | 新二进制不代表旧进程已加载 |

每次记录证据、改前值、预期、风险、验收和恢复方式，只维护当前文件。需要提交时运行`python3 scripts/check_source_tree.py`、`git diff --check`；固件镜像、SDK动态库、模型、日志、录像不入Git。提交/推送须由用户明确要求。
