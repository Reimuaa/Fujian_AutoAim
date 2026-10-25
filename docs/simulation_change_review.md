**第二份仿真修正文件对比与迁移评估**

评估日期：2026-10-24。正式工程：`D:\git_clone\sp-vision-2026-integration-archive`，比较起点提交：`8bb7251ff812d3ff37d214e9754d710284dd712b`。新候选：`D:\git_clone\Fujian_AutoAim-main_2`；上一份候选：`D:\git_clone\Fujian_AutoAim-main`。

评估阶段只更新此报告，未合入候选源码、配置或测试，未同步小电脑，未启动实车或有弹量仿真。该阶段C++离线验证直接读取候选源码，临时夹具在系统临时目录自动清理，二进制/对象文件仅写正式工程build/，输出仅写logs/。候选目录不作为新的正式工程。

当前PLL关闭。本次曾为共享接口兼容性复跑PLL夹具，范围超出了当前主要评估目标；已执行结果仅保留为遗留记录，不作为当前非PLL高速方案的采用条件，也不继续扩展PLL测试。后续结论以当前启用的非PLL链路为准。

**后续执行更新：用户选择的三项已在本机完成**

在正式工程原文件上适配N2及必要的N3调用传播、N4和N5，并补充对应回归。PnP保护接在现有1°高速批量路径；只校验输入数量/有限性和求解结果，不带入N1严格凸筛选、0.1°细化或候选的完整预测解检查。只读查询只替换高速整车已跳板的逐面迭代；10µs应用于两条显式高速整车分支，最大10次，耗尽不收敛时返回无效。静止/低速、现场配置、历史质量修复、P微调及固定两帧资格保留。

正式工程本机9项离线回归和`auto_aim` Release库构建通过；96个均值查询及2880组弹道与完整预测参考一致，原1215组1°批量差分及15项其他几何保持。新增故障注入覆盖solvePnP返回false、NaN/Inf、负深度、中心在前但角点在后及OpenCV异常；真实Tracker初始化/关联拒绝与后续恢复通过。9µs/50µs边界验证高速10µs与旧模式1ms隔离。本阶段未运行PLL专用测试，未同步小电脑或启动程序。实际实现与验证范围见`docs/status.md`、`docs/configuration.md`及`logs/three_changes_*.log`。下文建议、候选统计及候选测试结果均保留为此前评估记录，不代表本轮全量迁移。

随后按用户要求已将这三项及必要的原1°批量投影、初始化和单帧试验清理同步到`/home/rm/sp_vision`，小电脑`standard`编译、9项非PLL离线回归及本次diff检查通过，16个源码/测试文件逐项核对哈希。配置未传输，同步及验证期间现场配置哈希保持；没有启停程序。全树检查仍有既有固件备份工具的Git索引问题，未修改索引。部署记录见`docs/status.md`及`logs/three_changes_sync_review.json`。

**完整比较口径**

统一CRLF/LF后比较内容，忽略.git、build、logs、data、缓存及IDE生成目录；不把本机模型/SDK、旧研究目录或截图目录的缺失算作源码删除。

| 比较对象 | 内容不同的已有文件 | 候选新增文件 | 原有路径未包含 |
|---|---:|---:|---:|
| 新候选相对上一份候选 | 17 | 2 | 0 |
| 新候选相对当前正式工程 | 30 | 23 | 2 |

本轮真正增量只有前一行的19个文件；第二行其余差异包括上一轮未选用的功能和正式工程后来完成的修复。两种比较不能混为一谈。固件、相机、串口收发实现及模型权重没有本轮新增改动。

**新增能力、工作量及风险**

工时指单人适配与离线验证估计，不含用户现场采集和验收。N1/N2/N3及N6共享工作，不能逐行简单相加。编号也用于后面的完整文件清单。

| 编号 | 改动 | 改进地方 | 工作量 | 风险 | 建议 |
|---|---|---|---|---|---|
| N1 | 高速输入四边形检查 | 要求4个有限角点、无共线/重复边退化且为严格凸四边形，避免坏观测进入PnP。 | 1–2小时 | 低至中；严格凸不等于高质量，也不保证角点语义顺序正确。 | 优先考虑；与0.1°细化解耦 |
| N2 | PnP解有效性与异常隔离 | 检查solvePnP成功/有限旋转和平移、全部物理角点正深度、变换结果有限；局部捕获cv::Exception，返回失败。 | 2–3小时 | 中；void→bool需同步调用者和替身；非高速旧路径仍保留原行为；未做多解筛选/重投影阈值。 | 建议采用基础保护 |
| N3 | Tracker拒绝失败观测与计数 | 初始化和关联两处显式跳过失败解，记录失败尝试次数，避免坏解参与滤波；包括身份回退尝试。 | 1–2小时 | 低至中；跳过可能减少有效观测，计数不是唯一装甲板数，只有总数尚不能解释失败原因。 | 与N1/N2一起迁移 |
| N4 | 只读未来几何查询 | 显式高速四板逐面分支只推进CV均值并查询所选面，省去反复复制完整Target、传播协方差及辅助滤波器。 | 2–4小时 | 低至中；必须保持与完整预测相同的状态/时间/几何定义；不能替代Tracker或不确定度预测。 | 值得独立采用，先保留现有1ms容差 |
| N5 | 高速迭代1ms→10µs | 减小飞行时间固定点迭代残差；最大次数仍10，同步收紧共享解检查。 | 3–6小时 | 中；影响收敛、选面及边界资格；不是单纯提速，也不是现场延迟参数调整。 | 单独对照，暂不与N4绑定 |
| N6 | 新增及扩展回归 | 完整预测作为均值查询对照；弹道边界、坏输入、Tracker拒绝/恢复及bool替身适配。 | 2–4小时 | 低；测试需保留本机初始化、1°批量、历史质量及P微调检查，不能整文件替换；PLL夹具不纳入本轮验收。 | 随对应功能吸收 |
| N7 | 公开源码阅读与候选清单 | 汇总8个项目和时序、姿态、回放等研究方向。 | 0.5–1小时整理；实现另计 | 文档本身低；其余建议未实现/未验证，不能归为本轮收益。 | 作为计划参考 |
| N8 | 修正历史请求间隔说明 | 将1000ms额外限制标为历史，区分释放时间与真实出弹周期。 | 0.5小时以内 | 低；候选仍写100ms，正式版现为800ms，需按本工程实际状态修订。 | 只吸收概念与历史标注 |
| N9 | README/状态/配置说明 | 补充新接口、验证范围、数值容差定义和未部署状态。 | 0.5–1小时 | 低；有旧路径、其他仓库及过时参数，不能覆盖正式文档。 | 选择性更新 |
| L1 | 已具备：初始化、numeric、批量投影 | 正式工程已实现初始化、显式头文件及保持1°的批量投影；候选不构成新的同项收益。 | 0小时重复迁移 | 重复移植可能把本机1°路径换成未选用的0.1°细化。 | 保留正式实现 |
| L2 | 沿用上一轮：0.1°细化和完整解检查 | 可降低搜索离散误差/拒绝无效预测，但不是这轮新增；此前尚未授权合入。 | 细化3–6小时；完整解检查2–4小时 | 中；会改变结果及拒绝帧，保护开关与细化目前耦合。 | 继续单独候选，不作为N1/N4的前置绑定 |
| L3 | 沿用上一轮：Windows/资源/构建检查 | WRL、CMake、依赖构建、OBJ隔离、资源分类和模型/环境检查。 | 资源分类≤0.5小时；COM/CMake1–2小时；完整构建工具3–6小时 | 低至中；MSVC/vcpkg/Ninja缓存、ABI和DLL版本需适配。 | 按环境需要迁移 |
| L4 | 沿用上一轮：仿真入口、探针与统计 | BGR→RGB、共享帧诊断、有限单发、审计及成对报告。 | 颜色转换0.5–1小时；探针4–8小时；统计/状态机3–6小时 | 仿真中等；不等于实车完整链路或独立命中真值。 | 按仿真诊断需要使用 |
| L5 | 沿用上一轮：离线证据分析 | 成功写请求与外部确认的弹丸证据分开统计。 | 1–2小时 | 低；没有独立资料就不能推出真实出弹延迟或整轮命中率。 | 按需采用 |
| M1 | 旧配置差异 | 候选参数来自旧快照，并非本轮新调参。 | 0.5–1小时核对，不迁移旧值 | 高；涉及高速释放、预测、弹速、低速与门限。 | 保留正式配置 |
| M2 | 缺少正式历史质量稳定性修复 | 候选会回到短缺口清空历史、每过面单独覆盖查询点的旧规则，缺少历史保留/当帧资格分离及对应测试。 | 保留现版；若盲目覆盖需重新合并并回归 | 高；可能恢复频繁冷启动/门控跳变；候选旧规则并非本轮改进。 | 绝不回退这些改动 |
| M3 | 缺少高速−0.5°P微调 | 候选删除该配置/实现/测试，且文档不包含正式现场记录。 | 0小时采用；迁移时核对保留 | 高；会改变实际高速俯仰指令。 | 保留现版 |
| M4 | 当前非PLL回归保护缺失 | 候选缺少正式1°批量、质量稳定性、P微调等回归。 | 整合非PLL回归另计N6 | 中至高；不能删除当前启用链路的本机回归。 | 保留本机非PLL测试 |
| M5 | 已关闭PLL分支的旧夹具失败 | 候选PLL夹具缺少两个质量门变量，与当前非PLL运行故障无关。 | 本轮不投入修复；以后启用PLL时再评估 | 对当前采用结论非阻塞；仅保留遗留记录。 | 不继续扩展测试，不作为本轮采用条件 |
| M6 | 旧发布规则与文档 | 候选仍有助手推送/另一发布仓库的说明。 | ≤0.5小时审阅 | 中；与正式AGENTS规则冲突。 | 沿用本机规则，用户手动上传 |

推荐优先选择N1–N4的基础保护与轻量查询，连同必要回归约1–2个工作日；轻量查询先保持正式版1ms数值容差。N5单独对照，因为它已实际改变部分边界样本的有效性。PnP保护应接入现有1°批量路径，不能为了取得保护而被迫带入尚未选择的0.1°细化。

**本次实际复跑结果**

使用当前Windows本机依赖直接编译候选生产Solver/Tracker/Target/Aimer及其离线夹具，结果为 **8项中7项通过、1项失败**。完整记录：[simulation_candidate_review.log](D:/git_clone/sp-vision-2026-integration-archive/logs/simulation_candidate_review.log)。没有修改失败夹具来制造通过结果。

其中失败项属于当前未启用的PLL夹具，不属于本轮非PLL高速验收要求。该记录不能用来宣称当前运行链路存在同类问题。

| 验证 | 本次结果 | 能说明什么 |
|---|---|---|
| 只读几何查询 | 96个查询与完整Target预测一致；状态/协方差隔离及无效参数检查通过 | 支持在限定CV四板几何中替换冗余预测工作 |
| 弹道对照 | 2880组；1962有效、918无效；与同为10µs容差的完整预测参考一致 | 相同数值条件下的面号、时刻、点、双轴输出一致 |
| 改用更紧容差的行为差异 | 旧1ms有效1960，新10µs有效1962；新增4个有效解、减少2个 | 不能把N5当作完全等价的性能优化；解数不是请求数或命中数 |
| 最大飞行时间迭代残差 | 995.219µs→9.94381µs | 数值固定点残差缩小，不能推出实车精度提高100倍 |
| 局部预测微基准 | 完整旧参考约20.62µs，新Aimer约8.66µs | 约2.4倍的局部对照；测试同时改变查询实现和容差，不是单独N4的纯因果计时，也不是整帧提速 |
| 正常姿态/坏输入 | 216组正常姿态、6项排除目标一致性、20组坏输入/变换检查，以及真实Tracker拒绝初始化/关联与恢复均通过 | 支持基础保护正常工作；这20组不等同完整覆盖PnP返回false、cv异常或负深度等每个内部路径 |
| 正反旋转、模式/模型隔离 | 原高速预测两项、共享解检查一项、静止/低速隔离一项、EKF/ESKF/PLL参考一项均通过 | 候选自身的离线兼容检查通过；不是与当前正式链全量合并后的回归 |
| PLL事件资格测试 | 编译失败，缺high_speed_pitch_quality和high_speed_prediction_quality_open；fire_candidate因此无法初始化 | 上轮已发现的测试缺陷仍存在，不代表生产PLL算法本身已经损坏 |

姿态测试的0.1°细化与其约0.32–0.37ms→0.046–0.049ms计时属于上一轮继承实现；正式工程已经有1°批量投影，不能把该计时重复算作相对正式版新增的提速。理想夹具最坏角误差0.03°，非正交标定仍为2.47°，说明细网格不消除标定或PnP平移偏差。

候选文档记录的Linux构建/语法检查和历史仿真成绩本轮未独立复验；包内不含对应原始日志。本轮验证不等于完整Linux standard构建、现场命中率验收或实际完成用时改善。

**必须保留的正式工程差异**

以下为比较起点的正式配置；后续现场参数仍以唯一配置源configs/standard.yaml为准。

| 项目 | 正式工程 | 新候选 | 处理 |
|---|---|---|---|
| 高速基础补偿 | 335ms | 370ms | 保留正式值 |
| 低速基础补偿 | 320ms | 330ms | 保留正式值，不能改变低速行为 |
| 静止基础补偿 | 330ms | 330ms | 相同 |
| 高速Yaw门限 | ±0.1° | ±0.2° | 保留正式值 |
| 高速Pitch稳定等待 | 0ms | 30ms | 保留正式值 |
| 配置弹速 | 23m/s | 20m/s | 保留用户现场值 |
| 高速低电平释放 | 800ms | 100ms | 保留800ms；100ms并非本轮新优化 |
| 高速额外请求间隔 | 0ms | 0ms | 相同；1000ms只应作为历史记录 |
| 高速独立P微调 | −0.5°及实现/测试 | 无 | 保留正式实现与测试 |
| 单帧试验开关 | 已移除，固定两帧/最优帧 | 无 | 不恢复 |

正式历史质量门已经区分“成熟历史暂时保留”和“当前预测/观测资格”，有150ms短缺口、200ms观测边界及共同相位覆盖检查。候选仍用旧版：上下文短暂失效即清空，缺少保留与当帧否决分离、共同覆盖修复及对应回归。直接覆盖会丢失这些近期改动，可能重新出现频繁清空、重新成熟和门控跳变，不能将其视为本轮新方案。

**全部53个候选新增/不同文件**

“本轮修改/新增”以与上一份候选比较为准；“改/增”以与正式工程比较为准。每个文件对应的收益、工时及风险见上方编号；同组必须一起适配，不能把它们当成彼此独立的工时。

| 文件（点击打开候选） | 相对正式工程 | 相对上一份候选 | 实质差异及改进地方 | 工时/风险编号 |
|---|---|---|---|---|
| [AGENTS.md](D:/git_clone/Fujian_AutoAim-main_2/AGENTS.md) | 改 | 本轮未变 | 旧上传约定，允许助手推送；与正式规则不同。 | M6 |
| [CMakeLists.txt](D:/git_clone/Fujian_AutoAim-main_2/CMakeLists.txt) | 改 | 本轮未变 | 上一轮Windows目标、JSON链接与编译参数兼容修复，无本轮新增。 | L3 |
| [README.md](D:/git_clone/Fujian_AutoAim-main_2/README.md) | 改 | 本轮修改 | 新增轻量预测、10µs容差和PnP保护说明；仍带其他发布仓库及旧上传说明。 | N9/M6 |
| [configs/standard.yaml](D:/git_clone/Fujian_AutoAim-main_2/configs/standard.yaml) | 改 | 本轮未变 | 沿用上一轮旧参数并启用0.1°细化；对比现版缺少高速P微调，释放800→100ms等均是旧快照差异。 | M1/M3 |
| [docs/configuration.md](D:/git_clone/Fujian_AutoAim-main_2/docs/configuration.md) | 改 | 本轮修改 | 新增10µs/轻量查询说明；将1000ms限制标为历史，但仍写100ms释放，缺少正式版批量与质量门新说明。 | N8/N9/M1/M2 |
| [docs/dependencies.md](D:/git_clone/Fujian_AutoAim-main_2/docs/dependencies.md) | 改 | 本轮未变 | 上一轮Windows环境指南链接。 | L3 |
| [docs/field_measurements.md](D:/git_clone/Fujian_AutoAim-main_2/docs/field_measurements.md) | 增 | 本轮修改 | 本轮纠正历史1000ms描述，但100ms释放仍与当前正式800ms不同。 | N8/M1 |
| [docs/gestalt_fixed_target.md](D:/git_clone/Fujian_AutoAim-main_2/docs/gestalt_fixed_target.md) | 增 | 本轮未变 | 上一轮固定靶历史对照、计数审计及限制说明。 | L4 |
| [docs/open_source_review.md](D:/git_clone/Fujian_AutoAim-main_2/docs/open_source_review.md) | 增 | 本轮新增 | 本轮新增8个公开项目的阅读清单和后续候选建议；实际新增代码仅PnP相关，其余多为计划。 | N7 |
| [docs/status.md](D:/git_clone/Fujian_AutoAim-main_2/docs/status.md) | 改 | 本轮修改 | 新增PnP与弹道验证记录；缺少正式工程当前部署/历史质量/参数/1°批量等状态。 | N9/M2/M3 |
| [docs/windows_gestalt.md](D:/git_clone/Fujian_AutoAim-main_2/docs/windows_gestalt.md) | 增 | 本轮未变 | 上一轮Windows依赖构建说明。 | L3 |
| [io/gestalt/shot_trial.hpp](D:/git_clone/Fujian_AutoAim-main_2/io/gestalt/shot_trial.hpp) | 增 | 本轮未变 | 上一轮仿真预算单发/结算/提前准备状态机。 | L4 |
| [io/gestalt/win_capture.cpp](D:/git_clone/Fujian_AutoAim-main_2/io/gestalt/win_capture.cpp) | 改 | 本轮未变 | 上一轮ATL→WRL兼容改动。 | L3 |
| [scripts/analyze_shot_timing.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/analyze_shot_timing.py) | 增 | 本轮未变 | 上一轮离线成功请求与独立出弹证据分析。 | L5 |
| [scripts/audit_gestalt_counts.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/audit_gestalt_counts.py) | 增 | 本轮未变 | 上一轮原始WS及逐发计数审计。 | L4 |
| [scripts/build_gestalt_windows.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/build_gestalt_windows.py) | 增 | 本轮未变 | 上一轮固定依赖下载/校验/构建；不能原样替换当前缓存与环境。 | L3 |
| [scripts/check_runtime_assets.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/check_runtime_assets.py) | 改 | 本轮未变 | 上一轮模型/SDK类型筛选。 | L3 |
| [scripts/gestalt_frame_probe.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/gestalt_frame_probe.py) | 增 | 本轮未变 | 上一轮共享帧身份/推进检查。 | L4 |
| [scripts/gestalt_overlap_benchmark.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/gestalt_overlap_benchmark.py) | 增 | 本轮未变 | 上一轮冻结计划的成对对照；execute会运行仿真。 | L4 |
| [scripts/gestalt_probe.mjs](D:/git_clone/Fujian_AutoAim-main_2/scripts/gestalt_probe.mjs) | 增 | 本轮未变 | 上一轮本机WS遥测与可选控制，不是新增实车代码。 | L4 |
| [scripts/gestalt_series_report.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/gestalt_series_report.py) | 增 | 本轮未变 | 上一轮多轮统计、文件/版本核验。 | L4 |
| [scripts/gestalt_speed_report.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/gestalt_speed_report.py) | 增 | 本轮未变 | 上一轮用时比较与原生计数核验。 | L4 |
| [scripts/gestalt_trial_report.py](D:/git_clone/Fujian_AutoAim-main_2/scripts/gestalt_trial_report.py) | 增 | 本轮未变 | 上一轮逐发伤害/HP/结算统计。 | L4 |
| [scripts/requirements_gestalt.txt](D:/git_clone/Fujian_AutoAim-main_2/scripts/requirements_gestalt.txt) | 增 | 本轮未变 | 上一轮固定Windows依赖。 | L3 |
| [scripts/run_gestalt_live.ps1](D:/git_clone/Fujian_AutoAim-main_2/scripts/run_gestalt_live.ps1) | 增 | 本轮未变 | 上一轮仿真启动/停止/有限试验入口。 | L4 |
| [simulation/README.md](D:/git_clone/Fujian_AutoAim-main_2/simulation/README.md) | 改 | 本轮未变 | 上一轮场景区分与Windows环境链接。 | L4 |
| [src/gestalt.cpp](D:/git_clone/Fujian_AutoAim-main_2/src/gestalt.cpp) | 改 | 本轮未变 | 上一轮检测前BGR→RGB修正。 | L4 |
| [src/gestalt_vision_probe.cpp](D:/git_clone/Fujian_AutoAim-main_2/src/gestalt_vision_probe.cpp) | 增 | 本轮未变 | 上一轮视觉探针；仍不是standard完整资格链。 | L4 |
| [src/standard.cpp](D:/git_clone/Fujian_AutoAim-main_2/src/standard.cpp) | 改 | 本轮修改 | 本轮新增PnP失败次数诊断；继承完整解校验与细化接入，同时缺少正式历史质量门当前预测否决、保留逻辑及高速P微调。 | N3/M2/M3/L2 |
| [tasks/auto_aim/CMakeLists.txt](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/CMakeLists.txt) | 改 | 本轮未变 | 上一轮UTF-8与/FI参数修复。 | L3 |
| [tasks/auto_aim/aimer.cpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/aimer.cpp) | 改 | 本轮修改 | 显式高速逐面迭代使用均值几何查询；高速整车迭代终止容差改为10µs。 | N4/N5 |
| [tasks/auto_aim/aimer.hpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/aimer.hpp) | 改 | 本轮修改 | 本轮新增10µs常量并收紧共享检查；完整预测解校验来自上一轮，默认初始化正式工程已有。 | N5/L1/L2 |
| [tasks/auto_aim/face_prediction_residual.hpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/face_prediction_residual.hpp) | 改 | 本轮未变 | 缺少正式版150ms历史查询保留、短暂上下文恢复及多过面共同覆盖修复。 | M2 |
| [tasks/auto_aim/high_speed_fire_quality.hpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/high_speed_fire_quality.hpp) | 改 | 本轮未变 | 差异仅缺少正式版保留历史与当帧资格分离的说明，函数逻辑相同；不能将其算为新增质量改进。 | M2 |
| [tasks/auto_aim/solver.cpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/solver.cpp) | 改 | 本轮修改 | 本轮增加点集/解/深度/异常检查及bool返回；候选沿用上一轮0.1°细化，正式版的1°批量实现需保留。 | N1/N2/L2 |
| [tasks/auto_aim/solver.hpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/solver.hpp) | 改 | 本轮修改 | solve改为bool并抽出solve_pose；继承上一轮细化接口。 | N2/N6 |
| [tasks/auto_aim/target.cpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/target.cpp) | 改 | 本轮修改 | 新增只读predict_rigid_armor：CV均值推进并查询单个物理面，不传播协方差或辅助滤波器。 | N4 |
| [tasks/auto_aim/target.hpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/target.hpp) | 改 | 本轮修改 | 新增RigidArmorPrediction与几何查询声明。 | N4 |
| [tasks/auto_aim/tracker.cpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/tracker.cpp) | 改 | 本轮修改 | 两处检查Solver返回，失败候选不初始化/更新并计数；细化接入来自上一轮，<numeric>正式版已有。 | N3/L1 |
| [tasks/auto_aim/tracker.hpp](D:/git_clone/Fujian_AutoAim-main_2/tasks/auto_aim/tracker.hpp) | 改 | 本轮修改 | 本轮新增失败尝试计数；候选细化setter不能直接替换正式批量setter的含义。 | N3/L2 |
| [tests/gestalt_dependencies_test.cpp](D:/git_clone/Fujian_AutoAim-main_2/tests/gestalt_dependencies_test.cpp) | 增 | 本轮未变 | 上一轮无模型依赖检查。 | L3 |
| [tests/gestalt_models_test.cpp](D:/git_clone/Fujian_AutoAim-main_2/tests/gestalt_models_test.cpp) | 增 | 本轮未变 | 上一轮空白图模型加载/输出检查，不是检测精度验证。 | L3 |
| [tests/gestalt_shot_trial_test.cpp](D:/git_clone/Fujian_AutoAim-main_2/tests/gestalt_shot_trial_test.cpp) | 增 | 本轮未变 | 上一轮假属性/时钟状态机检查。 | L4 |
| [tests/test_ekf_only_prediction.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_ekf_only_prediction.py) | 改 | 本轮修改 | 本轮Solver替身适配bool返回。 | N6 |
| [tests/test_face_prediction_residual.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_face_prediction_residual.py) | 改 | 本轮未变 | 缺少正式工程新增约170行历史保留/覆盖稳定性测试。 | M2/M4 |
| [tests/test_high_speed_ballistics.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_high_speed_ballistics.py) | 增 | 本轮新增 | 本轮新增96次几何查询、2880组弹道对照和状态/协方差隔离检查。 | N6 |
| [tests/test_high_speed_fire_quality.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_high_speed_fire_quality.py) | 改 | 本轮未变 | 缺少正式工程高速P微调、固定两帧及当帧预测否决等测试。 | M3/M4 |
| [tests/test_high_speed_pose.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_high_speed_pose.py) | 增 | 本轮修改 | 继承上一轮姿态测试；本轮加入20组坏输入/变换、真实Tracker拒绝与恢复。 | N6 |
| [tests/test_high_speed_prediction.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_high_speed_prediction.py) | 改 | 本轮修改 | 本轮补充9µs通过/50µs拒绝边界；共享解检查来自上一轮，并缺少本机初始化检查。 | N6/L2/M4 |
| [tests/test_lightbar_search.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_lightbar_search.py) | 改 | 本轮未变 | 上一轮独立.obj、动态依赖/DLL路径及本地依赖前缀支持，无本轮新增。 | L3 |
| [tests/test_pll_event_timing.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_pll_event_timing.py) | 改 | 本轮未变 | 已关闭PLL分支的旧夹具缺少质量门变量；仅遗留记录，不作为本轮非PLL采用条件。 | M5 |
| [tests/test_shot_timing.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_shot_timing.py) | 增 | 本轮未变 | 上一轮离线证据统计测试。 | L5 |
| [tests/test_stationary_low_speed_restore.py](D:/git_clone/Fujian_AutoAim-main_2/tests/test_stationary_low_speed_restore.py) | 改 | 本轮修改 | 本轮Solver替身适配bool；候选用细化计数，正式工程用批量计数与拒绝ESKF开启检查。 | N6/M4 |

**正式工程的2个路径未随候选提供**

| 路径 | 意义 | 工作量及风险 | 处理 |
|---|---|---|---|
| [docs/simulation_change_review.md](D:/git_clone/sp-vision-2026-integration-archive/docs/simulation_change_review.md) | 本工程自己的评估记录 | 无算法收益，保留即可 | 本次在此文件更新评估，不创建带后缀报告副本 |
| [tests/test_batched_yaw_projection.py](D:/git_clone/sp-vision-2026-integration-archive/tests/test_batched_yaw_projection.py) | 已有1215组1°批量投影差分保护 | 删除会失去正式版等价性检查；迁移后还需保留或适配 | 不按候选缺失将它删除 |

**公开源码阅读文档中的15项尚未实现建议**

以下仅是新文档提出的后续方向，不能列作已完成改进。工作量是初步工程估计，不包含现场等待、额外设备或模型训练周期；本次未重新联网审阅文中8个外部仓库，未据它们保证性能。涉及执行/离膛延迟、弹速或固件协作的部分仍受用户既定范围约束。

| 未实现建议 | 潜在改进地方 | 初步工作量 | 主要风险/依赖 |
|---|---|---|---|
| 曝光/IMU设备时间映射 | 减少时间偏置和漂移的不确定性 | 3–7天，另需硬件配对证据 | 高；收帧/收包时间回归不等于曝光同步 |
| 逐发实际出膛证据关联 | 区分软件请求与物理出弹 | 0.5–2天工具/数据整理，采集另计 | 中；属于用户测量工作，不自动调延迟或弹速 |
| IPPE多解筛选 | 减少平面姿态错误分支 | 2–4天及真实帧对照 | 中至高；正深度/低重投影误差仍可能是错分支 |
| 弹速来源与边界审计 | 暴露入口与模型的有效范围/回退不一致 | 1–2天 | 中；先审计，不改用户弹速 |
| 已有角点细化器对照 | 定位高速拖影/遮挡造成的角点偏差 | 2–5天及标注数据 | 中；可能改坏角点并增加帧龄 |
| 物理尺寸角窗影子评估 | 解释固定Yaw门限拒绝的候选 | 1–3天及有效判伤尺寸 | 中；不能直接放宽现版±0.1°门限 |
| 请求/门控/机械等待分解 | 查清完成时间瓶颈 | 1–3天工具与日志分析，实际出弹证据另计 | 中；本机当前释放800ms，不沿用文档100ms |
| 原角点多帧固定中心联合优化 | 减少PnP三维偏差对模型的影响 | 1–2周 | 高；可观性、过拟合、计算负载与错误几何风险 |
| 噪声与滤波一致性校准 | 匹配不同距离/入射角的残差 | 3–7天及留出数据 | 中至高；无状态真值不能把内部NEES当真值校验 |
| 历史质量冷启动/接管等待分析 | 减少无收益的重新学习等待 | 1–3天分析 | 中；正式版已修短缺口稳定性，必须在其基础上继续 |
| 云台前馈/Planner或MPC | 改善确有响应瓶颈时的跟随 | 前馈3–5天；完整MPC/协议协作1–3周 | 高；当前14字节协议不能表达完整速度/加速度前馈 |
| 端到端帧龄/异步流水线 | 减少处理积压而非只提高FPS | 2–5天测量与原型 | 中；结果可能更旧，需逐槽保持原帧身份 |
| 模型与数据集对照 | 定位实际感知失败场景 | 2–5天评估；训练另计 | 中至高；交付中没有新权重或训练成果 |
| 有限队列原始输入回放 | 复现完整感知/姿态/资格结果 | 3–5天 | 中；录制会加负载，须记录丢帧与版本 |
| 正式入口模块化 | 降低多功能整合与回归成本 | 1–2周 | 中至高；先行为等价，不另建一套正式实现 |

此评估没有授权迁移上述任何未选择的功能；正式源码、配置、模型、固件、Git发布规则及用户手动上传流程保持当前工程状态。
