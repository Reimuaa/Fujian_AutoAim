**仿真修正文件对比与迁移评估**

比较目录：`D:\git_clone\Fujian_AutoAim-main`；正式工程：`D:\git_clone\sp-vision-2026-integration-archive`。评估日期：2026-10-24。

以用户提出比较时的本机工作文件为基准，统一 CRLF/LF 后，共发现 **24 个已有文件有内容差异、21 个新增文件**；正式 Git 跟踪路径没有缺失。不把本机运行模型、旧研究目录、UI 截图及构建/日志/采集数据的缺失视为源码删除。仿真目录没有被作为正式工程或整仓覆盖来源。

用户随后明确要求舍弃单帧试验开关：已在正式本机文件中移除该开关及旁路，保留高速 P 微调及现场参数。以下列出最初全部差异，并注明这项后续决定。其他候选功能尚未合入，未同步小电脑，未启动现场程序或仿真射击。

后续执行更新：用户已选择R1与R3。本机完成预测点初始化、显式`<numeric>`及高速批量重投影；保留原140个1°候选和误差比较，不带入候选的0.1°细化或完整预测解校验。其他条目仍为评估建议，原24+21文件清单是最初对比记录。现场参数以当前`configs/standard.yaml`为准，不将本报告比较期间的数值作为固定基线；本轮未调参、未同步小电脑。

**结论与逐项取舍**

真正新增的实车算法能力主要是高速姿态细化和完整预测解校验。`aimer.cpp`、`target.cpp`、固定中心模型、分面残差模型、成熟历史质量门、相机/串口实现均没有新改动；已有能力不能重复计为此次仿真收益。没有新模型权重或训练改动。

本报告保留比较时的参数快照与验证范围；当前运行值和部署状态以`configs/standard.yaml`及`docs/status.md`为准。

工作量为熟悉现有工程后的单人实现、适配及离线回归估计，不含用户现场采集与验收；关联项共享工作，不能逐行简单相加。

| 编号 | 改动 | 改进地方 | 预计工作量 | 风险 | 建议 |
|---|---|---|---|---|---|
| R1 | 预测点默认初始化、补 <numeric> | 消除首次读取未初始化字段和依赖间接包含头文件的问题。 | 0.5小时以内 | 低 | 已在本机实现 |
| R2 | 完整预测解校验 | 要求双轴/点有限、四板面号有效、迭代收敛、飞行时间与预测时刻一致；阻止无效解进入高速资格及分面学习。 | 2–4小时 | 中：可能增加拒绝帧、减少请求，须核对原因 | 建议先采用 |
| R3 | 批量重投影 | 保持原 140 个粗候选，用批量 projectPoints 减少重复矩阵转换和调用开销。 | 1–2小时 | 低至中：数值等价、畸变及非正交标定需回归 | 已在本机实现，保留1°步长 |
| R4 | 0.1° Yaw 细化及高速模式接入 | 降低原 1° 搜索离散误差；仅小装甲非基地/非前哨，在配置 Tracker 高速时启用；独立旧 Tracker 保持原解算。 | 3–6小时，另需现场数据 | 中：角点噪声/标定偏差仍在；上一帧模式驱动与分类耦合需观察 | 建议分开对照 |
| R5 | 新增回归及测试编译隔离 | 验证解校验、姿态与静止/低速隔离；独立 .obj 减少跨测试污染。 | 1–3小时 | 低：依赖库/DLL 路径与版本需适配 | 随对应功能采用 |
| R6 | 仿真 BGR→RGB | 使仿真检测输入符合实车相机/灯条颜色门约定。 | 0.5–1小时 | 低；只改仿真入口，改后需重新评估历史结果 | 建议采用 |
| R7 | Windows COM/CMake 兼容 | 去掉 ATL 依赖，修正 MSVC 参数并显式链接 JSON。 | 1–2小时 | 低；需重新编译仿真 | 可选采用 |
| R8 | Windows 独立依赖构建与冒烟测试 | 便于新电脑重建和定位模型/环境接入问题。 | 3–6小时，不含下载/冷编译等待 | 中：现有 build 为 MSVC/vcpkg，候选脚本使用 Ninja/自建依赖，可能缓存或 ABI 冲突 | 按需迁移 |
| R9 | 运行资源按类型筛选 | 仿真只检查模型，无需相机 SDK；默认全量检查保持。 | 0.5小时以内 | 低 | 可直接采用 |
| R10 | 仿真视觉探针、遥测与启动器 | 检查同帧姿态/身份/时龄，形成可观察的固定旋转靶诊断链。 | 4–8小时 | 实车无直接收益；仿真控制和资格逻辑独立，需检查版本/协议/默认参数 | 仅用于仿真 |
| R11 | 有限单发、统计审计、成对对照 | 把出弹、结算、有效受击、耗弹和完成时间分开；保留失败与未完成记录。 | 3–6小时，与探针共享部分工作 | 仿真中等：计数仍同源于游戏；提前准备并未在历史对照取得预设收益 | 保留诊断，默认不启用提前准备 |
| R12 | 离线请求/独立出弹证据分析 | 按 run/id 关联，统计软件时序和外部测量；不会用请求数冒充弹数，也不自动调参。 | 1–2小时 | 低：无独立测量时不能推出物理延迟或整轮命中率 | 建议采用 |
| R13 | 文档整理 | 保留版本、场景、证据定义和实验失败记录。 | 1–2小时 | 低；必须去掉旧路径、其他发布仓库、旧上传规则及过时参数 | 选择性吸收 |
| R14 | 候选旧参数与缺少高速 P 微调 | 无已证实收益；本机后续现场参数不能被旧快照覆盖。 | 0.5–1小时核对保留项 | 高：影响预测与非高速行为，破坏当前现场基线 | 不采用这些回退 |
| R15 | 候选 PLL 回归夹具 | 删除单帧夹具可以接受，但不能删除仍被 fire_candidate 引用的两个质量门变量。 | 0.5–1小时 | 候选测试已实际编译失败；不代表生产 PLL 本身故障 | 保留本机修复 |
| R16 | 移除单帧试验开关 | 按用户本轮要求删除开关和单帧旁路，固定使用两帧/最优帧资格。 | 已执行本机清理 | 低：清理前开关为 true，当前有效行为保持 | 已确认舍弃，未同步小电脑 |

**现场参数必须保留的差异**

| 项目 | 当前正式工程 | 仿真目录 | 处理 |
|---|---|---|---|
| 高速预测补偿 | 比较起点330ms；最终复核335ms | 370ms | 保留最终现值335ms，本轮未调整该参数 |
| 低速预测补偿 | 320ms | 330ms | 保留320ms |
| 静止预测补偿 | 330ms | 330ms | 保持 |
| 高速Yaw门限 | ±0.15° | ±0.2° | 保留±0.15° |
| 高速Pitch稳定等待 | 0ms | 30ms | 保留0ms |
| 配置弹速 | 23m/s | 20m/s | 保留23m/s |
| 高速独立P微调 | −0.5°，有实现及回归 | 无该实现/配置/测试 | 保留 |
| 单帧试验开关 | 原为true，当前已按用户要求删除 | 无该开关 | 固定两帧及最优帧资格 |

候选新增开关是 `xuc_high_speed_pose_refinement_enabled`，它与上述已舍弃的单帧资格开关无关。可将姿态细化设为缺省关闭，完成回放后再由用户选择启用。

**实际复跑结果与边界**

- 高速姿态：两个标定夹具各108组，共216组；每组包含原/新Solver对照。理想标定108组均改善，带取整非正交标定78组改善，全部满足原重投影目标不退化检查；各夹具另检查3种排除目标。
- 理想无噪声样本最大Yaw误差0.03°；非正交标定最大误差2.47°。0.1°是搜索步长，不是实车姿态精度承诺。平移仍由原PnP提供，深度偏差、实际倾角、曝光和角点噪声没有因此被解决。
- 本机合成样本每块板原解算约0.343/0.390ms，细化后约0.061/0.045ms。这是桌面机单板微基准，不是Linux小电脑整帧吞吐或命中率结果。
- 共享解校验14种无效输入拒绝通过；真实Tracker/Aimer的静止/低速恢复隔离、最终面选择及模型接管回归通过。静止/低速隔离测试792次旧链路比较、707次预测比较通过。
- 离线时序分析6项测试通过；仿真有限单发状态机的假时钟/假属性C++检查通过。
- 候选 `tests/test_pll_event_timing.py` 复跑失败：编译器报 `high_speed_pitch_quality` 和 `high_speed_prediction_quality_open` 未声明。候选删除了声明，但从主入口抽取的 `fire_candidate` 仍引用它们。本机清理保留这两个声明，只删单帧夹具。
- 单帧清理后的本机回归通过：固定两帧/最优帧及14项独立否决、高速P微调的非高速隔离、PLL双旋向事件几何及19项资格。源码树与diff检查通过。当前默认行为保持，代码尚未同步或重新编译为小电脑程序。
- 所有C++检查直接读取相应目录源码，只把二进制/对象文件写入正式工程 `build/`；临时夹具使用系统临时目录并清理。未在仿真目录建立工程副本或写入构建缓存。
- 本轮没有Linux完整重编译、模型精度验证、有弹量仿真或现场验收。候选文档中的历史228/250、94/100、96/100不属于本次新算法验证；交付目录不含这些试验的原始计划和日志，未能独立重验这些统计。
- 新探针虽复用Solver和Aimer，未复用standard全部成熟历史、Pitch、工作区、串口及模式资格。启动器默认200ms补偿、24.5m/s、+1.5°P偏置；这些参数不用于当前实车。其±8°是板面相位窗口，实车±0.15°是预测水平误差，定义不同，不能按数值比例比较或互换。
- 历史“预测副本速度归零”的固定中心试验已被主线联合固定中心模型替代，候选没有新实现；提前准备下一发的历史对照也未达到其预设采用条件，不能作为提速承诺。

**全部24个已有文件差异**

表中编号对应前述工作量和风险；这些文件属于同一功能时须一起适配，不能当作独立补丁分别叠加耗时。

| 文件（点击查看候选） | 全部实质差异 | 对正式工程的处理 | 评估编号 |
|---|---|---|---|
| [AGENTS.md](D:/git_clone/Fujian_AutoAim-main/AGENTS.md) | 撤回了当前“GitHub 上传仅由用户执行”的约定。 | 保留当前规则；不迁移。 | R13 |
| [CMakeLists.txt](D:/git_clone/Fujian_AutoAim-main/CMakeLists.txt) | MSVC 不再添加 GCC 风格标准参数；仿真显式链接 JSON；增加依赖、模型、单发状态机和视觉探针四个目标。 | 可选迁移 Windows 构建部分。 | R7/R8 |
| [README.md](D:/git_clone/Fujian_AutoAim-main/README.md) | 新增姿态细化/解校验交付说明；混入其他仓库地址并删除手动上传约定。 | 只吸收技术说明，保留正式仓库与上传规则。 | R13 |
| [configs/standard.yaml](D:/git_clone/Fujian_AutoAim-main/configs/standard.yaml) | 新增高速姿态细化开关，同时带回旧延迟、弹速、Yaw/Pitch 参数，缺少高速 P 微调。 | 只考虑新增开关；保留当前现场参数。单帧开关已按用户要求移除。 | R4/R14/R16 |
| [docs/configuration.md](D:/git_clone/Fujian_AutoAim-main/docs/configuration.md) | 新增高速姿态细化和完整预测解校验说明。 | 可吸收，修正为当前参数并注明验证边界。 | R13 |
| [docs/dependencies.md](D:/git_clone/Fujian_AutoAim-main/docs/dependencies.md) | 增加 Windows 环境指南链接。 | 随 Windows 工具迁移。 | R8/R13 |
| [docs/status.md](D:/git_clone/Fujian_AutoAim-main/docs/status.md) | 写入新 Solver/解校验状态，同时缺少本机后续调参和高速 P 微调记录。 | 选择性补充，保留现有记录；不能整文件覆盖。 | R13/R14 |
| [io/gestalt/win_capture.cpp](D:/git_clone/Fujian_AutoAim-main/io/gestalt/win_capture.cpp) | ATL 的 CComPtr 换成 Windows SDK 的 WRL ComPtr。 | 降低 Windows 编译环境要求；仅影响仿真采集。 | R7 |
| [scripts/check_runtime_assets.py](D:/git_clone/Fujian_AutoAim-main/scripts/check_runtime_assets.py) | 新增 --kind model/sdk 筛选，默认仍校验全部。 | 适合采用；仿真可单独检查模型。 | R9 |
| [simulation/README.md](D:/git_clone/Fujian_AutoAim-main/simulation/README.md) | 区分固定旋转靶新工具与历史前哨站结果。 | 适合补充说明，避免混用场景结果。 | R13 |
| [src/gestalt.cpp](D:/git_clone/Fujian_AutoAim-main/src/gestalt.cpp) | 仿真 BGR 图像在检测前转成 RGB。 | 适合仿真入口；不改实车相机适配器。 | R6 |
| [src/standard.cpp](D:/git_clone/Fujian_AutoAim-main/src/standard.cpp) | 新增上一帧高速分类驱动的姿态细化、预测完整性检查和诊断；候选缺少高速 P 微调。 | 只摘取新功能并保留 P 微调；单帧旁路已移除。 | R2/R4/R14/R16 |
| [tasks/auto_aim/CMakeLists.txt](D:/git_clone/Fujian_AutoAim-main/tasks/auto_aim/CMakeLists.txt) | Windows 增加 /utf-8 并修正 /FI 参数拼接。 | 适合随 Windows 构建兼容修复迁移。 | R7 |
| [tasks/auto_aim/aimer.hpp](D:/git_clone/Fujian_AutoAim-main/tasks/auto_aim/aimer.hpp) | 默认初始化 debug_aim_point；增加 high_speed_prediction_usable()。 | 初始化可直接采用；校验函数须连同两处调用和测试迁移。 | R1/R2 |
| [tasks/auto_aim/solver.cpp](D:/git_clone/Fujian_AutoAim-main/tasks/auto_aim/solver.cpp) | 批量投影 140 个粗候选，局部 0.1° 细化；保持平移、固定倾角和原重投影目标；无有效粗候选时回退。 | 最有价值的算法候选，需真实日志验证。 | R3/R4 |
| [tasks/auto_aim/solver.hpp](D:/git_clone/Fujian_AutoAim-main/tasks/auto_aim/solver.hpp) | solve 增加缺省关闭的细化参数，并声明批量误差/细化接口。 | 与 Solver 实现一并迁移，更新测试替身。 | R3/R4 |
| [tasks/auto_aim/tracker.cpp](D:/git_clone/Fujian_AutoAim-main/tasks/auto_aim/tracker.cpp) | 补 <numeric>；在初始化/关联解算中传入细化开关并记录诊断。 | 头文件可直接补；姿态调用随模式隔离迁移。 | R1/R4 |
| [tasks/auto_aim/tracker.hpp](D:/git_clone/Fujian_AutoAim-main/tasks/auto_aim/tracker.hpp) | 增加细化状态/诊断/setter；ESKF 启用时拒绝细化。 | 保护独立静止/低速 Tracker；须检查模式过渡。 | R4 |
| [tests/test_ekf_only_prediction.py](D:/git_clone/Fujian_AutoAim-main/tests/test_ekf_only_prediction.py) | Solver 测试替身适配新增 bool 参数。 | 迁移 Solver 时必须同步。 | R5 |
| [tests/test_high_speed_fire_quality.py](D:/git_clone/Fujian_AutoAim-main/tests/test_high_speed_fire_quality.py) | 候选缺少本机的高速 P 微调测试及单帧试验测试。 | 保留 P 微调测试；本轮已把单帧测试改为固定两帧/最优帧回归。 | R5/R14/R16 |
| [tests/test_high_speed_prediction.py](D:/git_clone/Fujian_AutoAim-main/tests/test_high_speed_prediction.py) | 增加共享预测解校验的 14 个无效输入案例。 | 适合与校验函数一起迁移。 | R2/R5 |
| [tests/test_lightbar_search.py](D:/git_clone/Fujian_AutoAim-main/tests/test_lightbar_search.py) | 每个测试单独存放 .obj；查找依赖库和运行 DLL；支持 build/deps/install。 | 可用，但保留当前 vcpkg 路径，避免混用库版本。 | R5/R8 |
| [tests/test_pll_event_timing.py](D:/git_clone/Fujian_AutoAim-main/tests/test_pll_event_timing.py) | 候选删掉单帧策略夹具，同时误删两个仍被生产语句引用的质量门变量。 | 候选实际编译失败；保留本机质量门变量。本轮只移除失效的单帧夹具。 | R15/R16 |
| [tests/test_stationary_low_speed_restore.py](D:/git_clone/Fujian_AutoAim-main/tests/test_stationary_low_speed_restore.py) | 适配 Solver 新参数；记录细化调用，验证旧 Tracker 不启用细化。 | 值得迁移，已离线复跑通过。 | R4/R5 |

**全部21个新增文件**

| 文件（点击查看候选） | 改进地方/用途 | 可用性与限制 | 评估编号 |
|---|---|---|---|
| [docs/field_measurements.md](D:/git_clone/Fujian_AutoAim-main/docs/field_measurements.md) | 请求/物理出弹/命中证据定义与 CSV 规范；含已过时的高速 1000ms 间隔描述。 | 修订后作为离线分析说明。 | R12/R13 |
| [docs/gestalt_fixed_target.md](D:/git_clone/Fujian_AutoAim-main/docs/gestalt_fixed_target.md) | 固定旋转靶历史实验、对照、审计和未完成事项。 | 保留历史边界；原始计划/日志不在交付目录内，不能重验其统计。 | R11/R13 |
| [docs/windows_gestalt.md](D:/git_clone/Fujian_AutoAim-main/docs/windows_gestalt.md) | 独立 Windows 依赖与构建说明，含原作者机器路径。 | 适配当前机器与现有 build 缓存。 | R8/R13 |
| [io/gestalt/shot_trial.hpp](D:/git_clone/Fujian_AutoAim-main/io/gestalt/shot_trial.hpp) | 有预算的单发、出弹回执、判伤结算、可选结算期准备下一发。 | 仅用于仿真状态机，不迁移为实车供弹控制。 | R11 |
| [scripts/analyze_shot_timing.py](D:/git_clone/Fujian_AutoAim-main/scripts/analyze_shot_timing.py) | 离线汇总成功请求写入、独立出弹 CSV、延迟分布、重复/冲突记录。 | 适合采用；不会自动修改延迟/弹速。 | R12 |
| [scripts/audit_gestalt_counts.py](D:/git_clone/Fujian_AutoAim-main/scripts/audit_gestalt_counts.py) | 直接从原始逐发与 WS 属性包独立复算计数/伤害。 | 适合仿真结果复核。 | R11 |
| [scripts/build_gestalt_windows.py](D:/git_clone/Fujian_AutoAim-main/scripts/build_gestalt_windows.py) | 固定源码 SHA256 的依赖下载/编译、虚拟环境、DLL 部署和离线检查。 | 先适配当前 MSVC/vcpkg 缓存；不要原样运行。 | R8 |
| [scripts/gestalt_frame_probe.py](D:/git_clone/Fujian_AutoAim-main/scripts/gestalt_frame_probe.py) | 只读 Windows 共享帧元数据，检查帧推进、视场和接管身份。 | 适合仿真诊断，依赖当前 v3 帧协议。 | R10 |
| [scripts/gestalt_overlap_benchmark.py](D:/git_clone/Fujian_AutoAim-main/scripts/gestalt_overlap_benchmark.py) | 固定计划的成对对照与报告；--execute 会执行仿真测试。 | 可用作实验组织工具，默认报告与执行模式需分清。 | R11 |
| [scripts/gestalt_probe.mjs](D:/git_clone/Fujian_AutoAim-main/scripts/gestalt_probe.mjs) | 本机 WS 遥测、角速度检查、可选姿态/目标旋转控制。 | 仿真专用；带控制选项时并非只读。 | R10 |
| [scripts/gestalt_series_report.py](D:/git_clone/Fujian_AutoAim-main/scripts/gestalt_series_report.py) | 多轮汇总、文件哈希核验、区间与完成目标分析。 | 改善可复核性；统计假设不能当实车效果证明。 | R11 |
| [scripts/gestalt_speed_report.py](D:/git_clone/Fujian_AutoAim-main/scripts/gestalt_speed_report.py) | 成对用时比较、原生计数前后核验和采用条件。 | 比较端到端耗时，而非只看命中比例。 | R11 |
| [scripts/gestalt_trial_report.py](D:/git_clone/Fujian_AutoAim-main/scripts/gestalt_trial_report.py) | 逐发伤害/HP/计数交叉核对；未全部结算时不报整轮命中率。 | 适合采用；仍以同一引擎遥测为依据。 | R11 |
| [scripts/requirements_gestalt.txt](D:/git_clone/Fujian_AutoAim-main/scripts/requirements_gestalt.txt) | 固定 OpenVINO/NumPy 等 Windows Python 依赖版本。 | 随独立构建工具迁移，不替换现场环境。 | R8 |
| [scripts/run_gestalt_live.ps1](D:/git_clone/Fujian_AutoAim-main/scripts/run_gestalt_live.ps1) | 仿真 start/stop/status/trial，状态读取重试、有限弹量、后台日志。 | Windows 仿真专用；默认 start 会接管仿真，应由用户启动。 | R10/R11 |
| [src/gestalt_vision_probe.cpp](D:/git_clone/Fujian_AutoAim-main/src/gestalt_vision_probe.cpp) | 共享帧检测/跟踪/预测、相机身份核验、有限单发试验和诊断。 | 可做诊断；没有复用 standard 的全部资格链。 | R10 |
| [tests/gestalt_dependencies_test.cpp](D:/git_clone/Fujian_AutoAim-main/tests/gestalt_dependencies_test.cpp) | 不含模型的 OpenCV/Eigen/YAML/日志/JSON/OpenVINO 冒烟测试。 | 环境交接有用，不证明检测精度。 | R8 |
| [tests/gestalt_models_test.cpp](D:/git_clone/Fujian_AutoAim-main/tests/gestalt_models_test.cpp) | 用空白图检查模型加载、解码、分类器输出范围。 | 定位模型/后端接入问题，不评估实景精度。 | R8 |
| [tests/gestalt_shot_trial_test.cpp](D:/git_clone/Fujian_AutoAim-main/tests/gestalt_shot_trial_test.cpp) | 假时钟/假属性测试结算、预算/目标结束清理和异常计数。 | 已独立编译运行通过。 | R11 |
| [tests/test_high_speed_pose.py](D:/git_clone/Fujian_AutoAim-main/tests/test_high_speed_pose.py) | 真实 Solver 的理想/非正交标定、畸变、回绕、噪声和排除目标测试。 | 值得采用；合成投影不等于真实相机验证。 | R4/R5 |
| [tests/test_shot_timing.py](D:/git_clone/Fujian_AutoAim-main/tests/test_shot_timing.py) | 离线分析的去重、冲突、未知标签和一请求多弹等六项测试。 | 六项已通过。 | R12 |

R1/R3已按用户选择在本机实现，1215组差分、15项其他几何检查、5项生产C++离线回归及auto_aim库构建通过；尚未同步小电脑。R2/R4等待用户后续选择，R6–R12可按仿真和诊断需要逐项加入。R14不迁移；R15保留本机有效夹具；R16已按用户要求清理。正式仓库、唯一配置源及用户手动上传约定继续沿用当前AGENTS.md。
