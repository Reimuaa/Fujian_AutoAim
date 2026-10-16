# 配置与标定状态

正式运行配置为configs/standard.yaml，标定工具配置为configs/calibration.yaml。启动器生成的logs/xuc-bringup/field_autoaim.runtime.yaml是缓存；不要将它作为另一份配置源。颜色选择保存在~/.config/sp-vision/field_autoaim_enemy_color。


## 2026-10-17步兵当前配置

当前步兵以`configs/standard.yaml`实际数值为准：Aimer高/低速延迟均为0.330s、旋转预测水平门限0.6°、配置弹速20m/s、弹道P轴固定微调已取消（0°）。ROI开启，区域x=420、y=300、600×600；YOLO26角点细化开启。`debug`和`detector_debug`关闭，`infantry_diagnostics_enabled`开启，云台控制及开火权限开启。`xuc_high_speed_phase_enabled: false`，所有旋转速度采用现有vehicle中速分支；独立事件延迟0.085s参数保留，不与330ms直接相加。高/低电平保持100/50ms，额外重装填间隔0ms。

固定P轴微调已取消，`xuc_ballistic_pitch_trim_deg: 0.0`；原有弹道补偿继续生效。旧注释中的3.2m/8cm对应之前-1.40°的试验背景。配置中的`pitch_offset: 6`是另一条原有参数，未在本轮更改。完整依赖与恢复方法见[快速恢复](recovery.md)。以下哨兵标定记录属于另一硬件目标，不能覆盖当前步兵参数。


## 内参：已完成

2026-09-17，海康MV-CS016-10UC、1440×1080。DFvision Q12-200-15，12×9方格、11×8内角点；格长15 mm由用户确认。29张图像全部识别，k3固定为0。平均逐点重投影距离0.0894 px，RMS 0.10465 px。
留出6张检查的RMS为0.0580～0.1672 px；每张仍拟合自身板姿态，不等于独立距离精度验证。23张训练与29张全量的fx/fy差分别为-0.0707%/-0.0569%。角点覆盖凸包约60.7%，极边缘未完全覆盖。
结果已写入两个现有配置。原始数据为data/calibration/sentry_intrinsics，详细质量记录为logs/calibration-quality.json。镜头焦距、对焦、分辨率或裁剪改变需重新核对。
内参采集：`./build/capture configs/calibration.yaml --intrinsics-only -o=data/calibration/sentry_intrinsics`；s保存、q退出。只保存图像，不生成姿态文件。
计算：`./build/calibrate_camera data/calibration/sentry_intrinsics -c=configs/calibration.yaml --no-display`。

## 姿态：回传已接通，双轴响应待确认

用户确认小Y IMU在UART3，大Y IMU在UART5。现场接收测试确定小Y为921600，大Y为115200。原固件错误地从USART1读小Y、将USART3用于裁判系统，且HI91长度拼接、CRC和字段读取有问题。
当前专用标定固件已修正并烧录：USART3由小Y IMU独占，标定模式禁用冲突的裁判收发；UART4 460800只读转发校验通过的小Y HI91完整帧，保留原始wxyz四元数、设备时间戳与CRC。串口DMA忙时丢弃新发包，不覆盖在途缓冲。没有恢复旧自瞄控制任务。
两路IMU均约100 Hz。小电脑5秒收到500帧、CRC错误0；capture --pose-check=3收到299帧、fresh=true、stationary=true。实际回传通过，但还不能以静止接收代替两轴运动及安装坐标验证。
此前约40秒只读记录中小Y摆幅13.49°、P摆幅10.39°，大Y/UART5的CRC有效姿态不变；这与大Y安装位置相符。SWD在运行中读取DMA缓冲可能跨帧，快照CRC失败不用于估算串口误码率。
协议参考：[HiPNUC官方解码器](https://github.com/hipnuc/products/blob/master/c/hipnuc/hipnuc_dec.c)。传感器原始轴和欧拉角约定不直接等同本车云台坐标，手眼采用原始完整四元数。

## 外参：尚未完成

当前P轴state=4/fault=3，位置超出保存目标±0.20 rad启动范围，保持失能；小Y遥控未放行。等待用户用支撑物托回原保存角度附近、手离开、摇杆回中后检查。没有扩大启动范围或更改保存目标32685。
恢复后先验证小Y姿态分别响应小Y和P，并确认IMU与相机刚性连接关系、轴方向和R_gimbal2imubody。若IMU只随小Y而不随P，需要正确的机构变换，不能直接套用现有手眼模型。
`calibration_pose_source: hipnuc`选择只读HI91接收；`calibration_pose_verified: false`会阻止外参保存，姿态验证完成后才修改。旧XUC解析保持原样，不与HI91混解。
采集工具支持`--pose-check=3`，仅检查姿态，不打开相机；正常采集要求姿态更新不超过100 ms、角速度小于2°/s且0.5秒内转角小于0.5°。断流、重复时间或未停稳均拒绝保存。数据目录保存来源标记hipnuc_uart3_wxyz，避免混入旧姿态样本。
全部现场角点与姿态确认后，固定棋盘、只转动相机采集data/calibration/sentry_handeye；计算并检查外参后再更新现有配置。当前R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody仍是旧安装值，不能视为哨兵实测。

用户之前提供的机器人中心坐标为X前/Y左/Z上，光心(15,0,50) cm、机身(10,0,50) cm、炮口(10,0,45) cm；这些记录尚未应用，不能替代相机—云台手眼结果。
运行相机工具前沿用现场MVS环境变量（/opt/MVS/lib/64等），不复制SDK。

## 步兵现场诊断开关

正式运行保持 `infantry_diagnostics_enabled: false`。需要排查时，在正式配置 `configs/standard.yaml` 中设为 `true`，由操作员启动。当前空载实验保留 `xuc_allow_shoot: true`，无弹时拨弹轮仍按原命令空转；这是空载运行，不是软件禁发模式。若另行需要软件禁发，再明确将该参数设为 `false`。诊断结束后将诊断开关改回 `false`。开关关闭时，逐帧 JSON、丢失画面、相机原始时间字段、频繁的 XUC 同步和 DRYRUN/B2DIAG 记录停止；每次启动仍保留一条 `[SYNC][XUC]` 供启动器确认下位机心跳，开火事件与错误也仍记录。

诊断摘要以 `[INFANTRY_DIAG]` 前缀写入当次 `logs/*.log`，约每 100 ms 一条，丢失边沿额外记录，最多 3000 条。摘要用 `run`、`frame_seq`、`frame_id` 和单调时钟纳秒关联相机帧、检测与跟踪、双轴指令和反馈、俯仰弹道补偿与开火门限。丢失瞬间的画面保存在 `data/infantry_diagnostics/`，最多 20 张，文件名和摘要的 `image` 字段对应。采集后保留当次日志和图片；实测距离、转速、测试时间及是否触及机械限位也一并记下。

逐请求诊断以 `[SHOT_DEBUG]` 输出，仅 `infantry_diagnostics_enabled: true` 时创建队列、后台线程和诊断状态。简洁模式继续调用原有 `XucSender::send()`，不采集这些字段、不构建 JSON、不入队或格式化。debug 模式在发送之后构建同一帧的 `request_snapshot`；它独立于上述 100 ms/3000 条采样，按 `run` 和 `id` 关联 `first_high`、`first_low` 及 `write_not_confirmed`。重试成功仍关联原请求；`frame_seq/frame_ns` 表示请求帧，`tx_frame_seq/tx_frame_ns` 表示实际发送帧。`accepted` 仅表示软件高低电平周期是否接受请求，不能当实际发数。

新记录使用最多 64 条待写记录的非阻塞有界队列，JSON 序列化及日志写入在后台完成。队列满或快照异常累计到 `queue_dropped_total`；日志 sink 错误累计到 `sink_errors_total`，正常退出时输出 `[SHOT_DEBUG_SUMMARY]`。debug 模式仍有采集、JSON 构建及后台日志负载，需要现场评估；简洁模式不创建这套诊断资源。

`planned_send_ns` 是事件分支计划发送时刻；`packet_prepare_ns` 是发送函数准备包时的主机时间，不能当精确的串口写入开始。`tx_write_complete_ns` 是完整写入调用返回时刻，`first_low` 的该时刻同时是软件低电平区间起点。精确写入开始、真实曝光中心、离膛、实际发数仍保留 `null`/`unknown`。Aimer 的实际迭代时刻和飞行时间现由只读 trace 暴露，定义见下文。快照分别记录 Aimer 输入/有效速度、直接俯仰和事件分支配置速度；反馈字段单位仍未证实，最新 RX 时间与速度不是原子配对，不能把它当已验证的弹速数据年龄。

开启诊断后，`[FIRE_GATE_SUMMARY]` 每 5 秒累计各级门限作为首个阻断条件的帧数、时长和最长连续时长；逐帧摘要含完整门状态。`[SHOT_TX]` 用 shot_id 区分请求与串口写入成功，记录写入开始、写入返回的主机时间及实际打包角度；写入返回不代表下位机接收或子弹离膛。`[SHOT_FEED_EST]` 用明确的拨弹电机转子RPM积分估算转角，保留历史4.5圈诊断阈值；该阈值基于旧8弹位、36:1假设，不能直接用于当前omni硬件的完成计数，须单独核实。`projectile_exit` 始终为 `unknown`。

2026-10-13起按用户提供的omni-code回传协议解析30字节包。完整诊断新增`[SHOT_DEBUG] kind=shooter_feedback`，按相机帧批量输出期间收到的全部回传样本：`host_rx_ns`（主机解析时的steady-clock纳秒）、`feedback_sequence`（回传序号，不是发数）、`fire_flag`、`fire_tick_ms`（下位机1kHz FreeRTOS tick）、`friction_current_raw`（一路摩擦轮原始CAN电流反馈，未换算安培）、`feeder_speed_rpm`（拨弹电机转子RPM）。接收暂存最多256个样本，溢出计入`samples_dropped_total`；现有异步日志队列丢弃另见`queue_dropped_total`。简洁模式不采集这一波形、不积分转角、不构造或写入新增日志。

下位机`fire_flag/fire_tick_ms`由两路摩擦轮电流绝对值和超过3000判定产生，并非经独立测速/光电验证的出膛事件；启动或加速也可能触发，不能直接拿它计算真实发射延迟。主机与下位机时钟尚未对齐，不直接相减。现有30字节包没有摩擦轮实时转速或拨弹编码器角度。

## 旋转目标中心过程噪声

Tracker 的中心状态仍采用恒速模型。`tracker_center_acceleration_variance` 是普通状态的中心加速度方差；当 `tracker_gyro_center_process_noise_enabled` 开启、YawESKF 已可信且角速度达到 `tracker_gyro_center_rate_threshold_rad_s` 时，改用较小的 `tracker_gyro_center_acceleration_variance`。这只降低旋转装甲板 PnP 深度偏差造成的虚假中心加速度，不冻结中心位置或中心速度。完整诊断中的 `tracker_center_acceleration_variance` 记录每帧实际采用的值。

## 空载连续预测诊断

空载实验保留原有发射权限、450/550 ms 软件周期、1.5°中速门限、85 ms 延迟和配置弹速；由操作员确认无弹并自行启动。不开实弹不等于 `shoot=0`：此实验仍可输出原有开火命令并让拨弹轮空转。

仅 debug 模式在原发送完成后，将每个处理帧的 `prediction_frame` 放入现有最多64条待写记录的异步队列，不依赖开火请求或上述100 ms/3000条摘要。简洁模式不创建连续诊断状态、不提取trace、不查询额外IMU历史、不构建JSON或写这些记录。Aimer的可选trace只观察现有迭代，不额外预测或选板，也不改变输出、门限、等待或发送逻辑。

`native_prediction_ns` 是最后一次实际用于选板和未来位置的预测时刻；`native_flight_time_used_s` 是该次预测使用的上一次飞行时间，`native_flight_time_s` 是在该未来点重新计算的最终飞行时间。两者并非强行相等，`native_iteration_converged`单独记录迭代是否达到原有1 ms收敛条件。逐请求`request_snapshot`升级为schema 2，同样记录这些时刻及未来板编号。

连续记录关联`run/frame_seq/track_epoch/native_face_id`；跟踪重新初始化、NIS重置、发散、目标消失后重建或身份/类型/板数变化时更换epoch。板编号是跟踪器相对编号，不是独立验证过的物理身份。`observations`只取实际参与当前更新的已求解PnP观测，包含板编号、世界位置和朝向，不拿未来滤波状态充当参考。`state_at_frame`顺序为cx,vx,cy,vy,cz,vz,body_yaw,omega,r1,r2_delta,height_delta。

最终输出和实际打包角度分开保存，保留高电平期间指令锁存的影响。反馈姿态从同一带时间戳的IMU历史样本获取，记录转换系数，离线还原到原始协议坐标后与打包角度比较；不将分别更新的原始角度原子量和另一时刻的RX时间拼成一对。`capture_us`、队列丢弃和sink错误用于评估debug负载；强制退出可能缺少末尾记录，计数为零不能证明退出前完全落盘。

离线分析：`python3 scripts/analyze_prediction_debug.py logs/<当次日志>.log --output logs/prediction_accuracy.json`。默认仅使用未来时刻前后各不超过25 ms的同epoch同板原始观测插值，禁止外推、跨重置或用另一块板代替；角度按圆周插值。缺失、未收敛、对应不明和日志提前结束分别计数。输出水平/俯仰/空间/相位残差、按run/分支/旋向/板分组、同时刻原始角度跟随残差、中心估计速度及采集耗时。它只评估原生Aimer的视觉一致性，不等于实弹命中率、独立真值、真实出弹延迟或高速事件预测精度。

先采静止靶跟随，再采固定云台看原地旋转，最后采正常跟随旋转；当前距离与中速先做左右旋各两段30—60秒。对比使用同一套配置，先分析再单项修改。当前录像调用保持原样关闭，原始数据仅使用data/，不直接启用旧Recorder的records/路径。

## 旋转误差分解诊断（第一步）

`prediction_frame` schema 2 在原有 debug 分支中补充实际用于求解的完整 IMU 四元数、Solver 的世界旋转矩阵、图像尺寸、检测角点及置信度、关联代价与纠正后板型、已接受观测的 PnP 优化前后 yaw、本帧更新前的整车状态和最终限速前后角度。初始化、重置和发散帧不提供可比较的更新前状态；平衡分支未初始化的 `yaw_raw` 不读取，记录为 null。标定上下文从当前已加载 YAML 提取，在首个记录及每60帧重复，以避免首条日志丢失导致整轮不可复算。上下文随日志保存，不作为另一份运行配置。检测、关联及观测证据各最多8条并记录总数和截断标志；不截断实际算法输入。角点来自跟踪时使用的图像坐标，不从录像重新检测。

新增采集全部在 debug 判定内部、原发送完成之后执行，不增加在线 PnP、预测、选板、发送或硬件访问。Solver、Tracker、Target、Aimer、云台和开火协议保持原样；简洁模式没有这些新增数据的提取、JSON 构造或日志负载。debug 仍有额外 CPU、内存和磁盘负载，必须通过 capture_us、帧间隔、图像到串口耗时和队列计数实测，不能用简洁模式无新增负载推断 debug 无影响。

分析命令：`python3 scripts/analyze_prediction_debug.py logs/<当次日志>.log --start 16:56:56 --pnp-replay --output logs/prediction_accuracy.json`。`--start`/`--end` 可用 HH:MM:SS 或完整日期时间；通常不指定结束时间。日志在时间筛选前读取标定上下文。PnP 复算仅依赖离线 Python OpenCV/NumPy，默认每10帧复算一次，`--pnp-stride 1` 可复算全部；其余模型和预测统计使用所有符合窗口的帧。旧日志、缺少角点或标定、标定冲突、无效姿态、截断和缺失对应板均明确排除，不能用当前 YAML 或其他板替代。

报告 `attribution` 区分更新前/更新后整车几何与当前 PnP 的残差、未来总误差与当前误差之差、不同板/观察角度/单板双板参与更新/身份回退分组、双板等效车体相位差，以及限速前后、最终输出与实际打包的差值。观测数量仅表示参与更新数量，不能等同真实可见板数；同时看到两板时 primary_face 来回变化本身也不能证明错误切板。更新后模型已经使用当前 PnP，不能作为独立真值。输出与原生预测的差包含有意的中心跟随和零位补偿，不能直接当瞄准误差。

`pnp_replay` 离线检查 IPPE 两个候选解、优化前姿态复现、现有固定倾角 yaw 网格搜索复现与重投影像素误差。小重投影误差不能证明深度/朝向真实准确；复算重现只能证明输入与实现一致，不能直接判断应该选择另一解。首阶段只提供归因证据，不自动调参或将离线解回灌控制。450/550 ms 软件周期、85 ms 延迟、1.5°门限、发射权限和参数源保持现状。

旧 schema 1 日志可按完全相同的 run/frame_seq 从已存在的 `[INFANTRY_DIAG]` 采样记录补取更新前状态，来源和样本数单独列出；不跨帧插值、不覆盖 schema 2 的明确 null。`paired_current_*` 仅比较同时有更新前后状态的同一组观测，避免样本集合不同造成误判。旧日志没有角点和标定上下文时仍不能复算 PnP。`tests/test_prediction_debug.py` 在小电脑包含正式 C++ Solver 的18组离线样本与 Python 复算一致性检查；二进制只写入 build/，临时夹具自动删除，不访问摄像头或串口。

## 双板共同中心离线几何验证

`scripts/analyze_prediction_debug.py` 的报告新增 `dual_geometry`，只在离线运行，在线简洁/debug模式均无新增采集或计算。只使用可信、IMU有效、未截断的四板目标同帧有效观测；每对不同板单独计算。`optimized_yaw_geometry` 使用现有优化yaw，`raw_yaw_geometry` 使用记录的原始yaw，缺失时明确排除，不互相补值。

位置连线与更新前记录半径给出共同相位和位置中心；每板的位置、各自yaw与相同半径给出各自中心。报告中心闭合差及其视线径向/横向分量、两板距离对模型几何的差、位置相位对各板yaw的差，以及中心对更新前中心的偏差。所有几何依赖当前半径、观测和相对板编号，不是独立测量真值。位置中心的帧间标准差仅描述波动，不能当真实平移速度。

从两板朝向反推独立半径时，`abs(sin(yaw_left-yaw_right)) < 0.1` 视为退化，只排除该项半径解；其他几何仍报告。负半径保留并标识，不裁剪成可用解。原始yaw的退化也由该值及缺失解字段明确表示。

`constant_center_holdout` 是固定中心假设检验：每个run/epoch内，仅用双板帧，交替1秒时间块训练/留出；至少各6帧，拟合Cx、Cy和奇偶两种半径。奇异系统、缺失yaw及非正半径明确报告；留出样本不参加拟合。报告拟合残差和同一批留出观测对应的现有更新前模型残差。该检验假设靶车无平移，不能用于一般运动场景的优劣判断；时间相关性与共享标定仍存在。离线拟合不回灌控制，不提供实际命中率。

按装甲面/观察角度/旋向/身份回退分组的指标与相关性用于核查相位、中心及半径更新之间的关系，不自动判定因果。`pair_samples` 保存逐帧配对证据，终端省略大体积明细，完整结果写入指定JSON。旧日志缺失更新前半径或原始yaw时，各自项目排除；不从当前配置补历史数据。验证命令仍为 `python3 -m unittest discover -s tests -p test_prediction_debug.py`。


## 中速四板预测使用同源整车 EKF

用户批准后，standard 仅在新鲜、可信的 vehicle 模式选择 `AimGeometry::whole_ekf`。Aimer 在本帧入口确认四板、非前哨站、当前旋转速度超过 decision_speed、原始整车状态有限且两种半径有效；不满足时保留已有路径。选中后，本帧所有弹道迭代固定使用同一来源，中心、速度、半径、相位和角速度来自整车 EKF；不把 ESKF 相位拼入整车几何。Target 新增明确几何接口，原 ekf_x/armor_xyza_list 及 Tracker 关联和更新不变。

ESKF 继续参与旋转识别、可信度、模式切换和原有高速路径。静止/直接装甲板、三板与前哨站保留原路径；高速模式调用已有几何路径。85 ms 延迟、1.5°门限、450/550 ms周期、开火权限、俯仰、PnP、噪声与中心跟随控制律不变。几何改变会改变选板和开火资格，加减速时整车恒角速度模型仍可能退化。

debug 的 prediction_frame/shot 快照新增 `native_geometry_source`，值为 whole_ekf_rigid 或 existing；prediction_frame 另记 `whole_ekf_state_at_frame`，保留原有组合 `state_at_frame` 用于识别语义。新增提取与 JSON 仅在 infantry_diagnostics_enabled 内执行，简洁模式不运行影子预测或新增诊断日志。

离线验证：`python3 tests/test_aimer_prediction_trace.py` 编译真实 Target/Aimer，验证原路径、双滤波分离后的同源预测、顺逆旋转、弹道迭代和简洁/debug指令一致。加 `--replay-log logs/<日志>.log --replay-report logs/prediction_accuracy.json` 可将实际 C++ 选板/弹道回放合入当前报告。回放使用已记录的基准预测时刻/速度，通过下一帧原始先验逆解 CV 转移恢复后验；只评分连续 epoch 的同板未来观测，不跨缺失外推。该回放固定既有关联，不重跑 Tracker；假定稳定旋转已 jumped，只比较水平资格门，不模拟发射周期、云台反应或实际弹丸，不能推算命中率。用户自行重启并进行现场测试。

## 2026-10-11 下午参数试验后的当前值

用户要求仅将 `xuc_gyro_predicted_yaw_tolerance_deg` 从1.5改为0.5，没有新增中心窗口调度代码。连续两帧规则保留。随后只试验Aimer的 `high_speed_delay_time` / `low_speed_delay_time`：85ms → 105ms → 60ms → 恢复85ms。当前两项均为0.085；高速事件独立参数 `xuc_event_fire_delay_s` 始终为0.085。弹速仍使用未实测确认的28m/s，其他控制与发射参数不变。

预测包括图像时间戳到Aimer当前时刻的实际耗时、配置基础延迟和迭代飞行时间。15ms IMU对齐量只用于姿态查询，450ms高/550ms低只定义请求保持与重新请求；不能把这些时间全部累加成离膛延迟。参数修改由用户重启后加载，不需要重新编译。105ms/60ms的实弹优劣没有完成量化确认，恢复85ms是用户选择，不代表已标定真实离膛延迟。
