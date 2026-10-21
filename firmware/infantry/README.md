# 步兵固件：架构、编译、烧录与调试

核对日期：2026-10-11。原步兵STM32F405RGT6正式源码仅本目录。车上最近实际烧录来源仍未知，下文描述仓库源码。哨兵使用`../sentry/`，不能混刷。

本轮只读核对并整理文档，没有编译或烧录固件、读取Flash、暂停或复位主控。以下操作步骤供你接手使用，尚未在本轮接板验证。

## 1. 文件与任务架构

| 文件（相对本目录） | 职责 |
|---|---|
| `STM32F405/STM32F405.cpp` | HAL/外设初始化、任务入口 |
| `STM32F405/taskslist.cpp/.h` | FreeRTOS任务，输入到运动/供弹的整合 |
| `STM32F405/xuc.cpp/.h` | 流式协议解析、CRC、新鲜度、反馈 |
| `STM32F405/xuc_aim.cpp/.h` | XucYawController、XucPitchController、XucAutoFireGate |
| `STM32F405/control.cpp/.h` | 云台/底盘与`CONTROL::SHOOTER::Update`供弹状态机 |
| `STM32F405/RC.cpp/.h`、`imu.cpp/.h` | DR16、IMU解析/严格CRC/新鲜度 |
| `STM32F405/motor.cpp/.h`、`HTmotor.cpp/.h` | DJI/达妙电机 |
| `STM32F405/can.cpp/.h`、`usart.cpp/.h` | CAN/UART底层 |
| `STM32F405/label.h`、`FreeRTOSConfig.h` | 控制宏、硬件与RTOS参数 |
| `CMakeLists.txt`、`cmake/arm-none-eabi-gcc.cmake`、`linker/STM32F405RG_flash.ld` | 交叉编译、内存布局 |

| 外设 | 源码映射 | 用途 |
|---|---|---|
| CAN1 | PA11/PA12，1Mbps | 摩擦轮、拨弹、达妙pitch |
| CAN2 | PB12/PB13，1Mbps | 底盘、yaw |
| USART2 | 921600 | CH010云台IMU |
| USART3 | 100000 | DR16 |
| UART5 | 9600 | 电源/裁判相关 |
| USART6 | PC6/PC7，460800 | 上位机XUC |

接线以实物原理图/丝印核对，不能套用哨兵IMU映射；本表不是板端插座针序。

~~~text
USART6 → xuc.PollReceive → rc.Update → ApplyBoundedXucYawControl
→ pantile.Update / chassis.Update / shooter.Update → CAN → 电机
~~~

ControlTask/DecodeTask使用vTaskDelay(5)，实际周期还含执行与调度；MotorUpdateTask配置2ms、CAN任务1ms、XucTask反馈10ms。周期不是完整发弹延迟。

## 2. 当前源码行为与现场未知项

旧README的“Phase 2A、±1°、pitch/发射不收命令”已过时。当前包含Phase 2D.5及后续双轴/供弹逻辑：

- AUTO源于`rc.s[0]==MID && rc.s[1]==DOWN`；现场拨杆对应和实际固件须核实。
- yaw保留RC、回中、严格IMU、命令新鲜度等门；目标相对解锁点±25°、实际超过28°锁止。
- pitch以AUTO进入时IMU/电机位置为锚映射目标，有限速和多处机械限制，不凭单处常量认定全行程。
- AUTO可保持摩擦轮运行；供弹还需要跟踪、pitch、control/shoot、转速和无故障。
- 两轮达到目标90%后，FireGate要求连续稳定200ms；SHOOTER另有每次完成后600ms AUTO间隔、单步供弹及堵转处理。不能理解为每次固定等待200+600ms。
- 当前6500RPM是摩擦轮设定，不能证明28m/s。拨弹步进完成计数不是离膛计数。

上位机450ms高/550ms低与实际固件是否一致仍须核实；长高电平可能在下位机稍后满足条件时才触发，不能先改协议或直接刷本仓库固件代替核验。

当前`taskslist.cpp::XucTask`复用20B反馈字段：

| 字段 | 本仓库含义 |
|---|---|
| mode | 操作者选择AUTO为1，不代表全部运动门已开 |
| robot_id | 运动门状态位 |
| bullet_speed | 拨弹M2006实测RPM |
| bullet_count | 两轮RPM各1字节，40RPM/LSB |
| imu_pitch/imu_yaw | 严格IMU角度换成rad |

车上字段未证明采用此表，不能按字段名当成弹速/发数。下一次真正烧录应把源码状态、构建清单、镜像SHA256、硬件和验证结果写入当前工作日志，不新建工程副本或归档标签。

## 3. Windows只编译、不连接硬件

现有入口`tools/build_firmware.ps1`：

| 依赖 | 固定约束 |
|---|---|
| GNU Arm Embedded GCC | 10.3.1 20210824，`C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10` |
| STM32CubeF4 | v1.24.1，提交`b5abca20c9676b04f8d2885a668a9b653ee65705` |
| 旧HAL | Cube v1.14.0，提交`b6e8197518e97d3fe9243fe49a692033b9d7d734` |
| CMake/Ninja/Git | 脚本使用cmake --fresh及Ninja，须支持并在PATH可调用 |

Cube提供CMSIS、启动文件和FreeRTOS；HAL单独用旧版，兼容`CanTxMsgTypeDef/HAL_CAN_Transmit`，不能直接升级。

本轮Windows检查：CMake可用，脚本指定GCC不存在，Ninja/OpenOCD不在当前PATH，默认firmware/toolchains SDK目录也不存在。先安装/定位指定依赖再执行；脚本没有任意-GccRoot参数。

在根目录PowerShell执行，下列SDK目录是示例，换成你实际准备的目录：

~~~powershell
.\firmware\infantry\tools\build_firmware.ps1 `
  -CubeRoot 'C:\SDK\STM32CubeF4-v1.24.1' `
  -HalRoot 'C:\SDK\stm32f4-hal-cube-v1.14.0\Drivers\STM32F4xx_HAL_Driver' `
  -BuildDirectory 'D:\git_clone\sp-vision-2026-integration-archive\build\firmware\infantry' `
  -Configuration Release
~~~

**显式指定BuildDirectory**：旧默认firmware/infantry/build-arm不符合当前产物只放根build的约定。本轮未改脚本。

产物`build/firmware/infantry/stm32f405_xuc_v2.elf/.bin/.hex`及map/build-manifest。脚本验证依赖提交、符号、初始SP/复位向量并输出哈希。Flash起点`0x08000000`、布局1MiB、初始SP`0x20020000`；ELF/HEX带地址，BIN不带。镜像不提交Git。

旧VisualGDB入口是FreeRTOS.sln与STM32F405/STM32F405.vcxproj；本轮未验证其工具链、输出和探针。若使用，也需输出至根build并核对xuc/xuc_aim等实际源文件。本指南优先使用可审阅的独立脚本。

## 4. 现场烧录

烧录/复位会改变主控状态。先由操作员停止上位机、清空弹丸并隔离执行机构动力，逻辑板按硬件要求供电；确认目标是步兵F405及对应镜像。

SWD需要SWDIO、SWCLK、GND、目标电压参考，必要时NRST；芯片信号PA13/PA14，插座针序以实物为准。不要给已供电板重复供电。下列模板假设**CMSIS-DAP**，实际ST-Link须选对应适配器配置。

OpenOCD安装且PATH可用后：

~~~powershell
openocd -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32f4x.cfg `
  -c "adapter speed 1000" `
  -c "program {D:/git_clone/sp-vision-2026-integration-archive/build/firmware/infantry/stm32f405_xuc_v2.elf} verify reset exit"
~~~

这是实际写Flash、校验、复位运行命令，本轮未执行。检查退出码和Verified OK等校验结果；连接成功不代表烧录成功。首次运行继续核对遥控、IMU、电机方向、限位和协议。

若使用已安装的STM32CubeProgrammer和ST-Link，选择SWD，加载对应HEX/ELF；BIN需设与链接布局一致的`0x08000000`。不盲目执行读保护解除、Option Bytes或解锁来解决连接问题。

固件备份脚本不纳入源码分支，不随工程上传。读Flash会reset/halt并扰动运行，需另行安排。

命令语义见[OpenOCD官方烧录文档](https://openocd.org/doc/html/Flash-Programming.html)，探针选项见[官方适配器配置](https://openocd.org/doc/html/Debug-Adapter-Configuration.html)。

## 5. 源码调试

优先遥测；运转中断点暂停会改变控制与发射时序，不能用单步时间测延迟。

需断点时，在同一build目录用`-Configuration Debug`重新构建（-Og -g3）。符号必须匹配实际镜像，必要时安排烧录Debug镜像。

终端一：

~~~powershell
openocd -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32f4x.cfg `
  -c "adapter speed 1000"
~~~

终端二：

~~~powershell
& 'C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin\arm-none-eabi-gdb.exe' `
  'D:\git_clone\sp-vision-2026-integration-archive\build\firmware\infantry\stm32f405_xuc_v2.elf'
~~~

已安排停机调试后，在GDB中：

~~~gdb
target extended-remote localhost:3333
monitor reset halt
break main
continue
~~~

按问题在ControlTask、XucAutoFireGate::Update、CONTROL::SHOOTER::Update观察命令新鲜度、各门、角度目标/反馈、两轮RPM、拨弹状态和计数。`load`会写Flash，不是普通观察；结束/恢复由操作者决定。[OpenOCD官方GDB说明](https://openocd.org/doc-release/html/GDB-and-OpenOCD.html)

本轮未验证VS Code launch配置；以后按实际探针配置Cortex-Debug，底层仍是同一ELF/OpenOCD/GDB，不复制其他MCU配置直接接板。

## 6. 本次接手的阅读主线

先读上位机standard的vehicle门控、io/xuc的保持周期，再读主控xuc_aim和control的等待/供弹逻辑。先确认实际固件，避免在未运行的源码上调参。

查“前两发中、后续独立单发偏/疑似发晚”时，区分预测窗口、串口请求、下位机允许供弹和实际出弹；不以精确离膛传感器为前置。85ms该改多少尚无结论。新增上位机诊断仅放Debug，简洁模式不增加采集、格式化和影子计算。
