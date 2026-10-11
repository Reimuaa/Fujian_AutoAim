# 恢复当前步兵自瞄

此发布保留用户2026-10-11晚间认可的版本。源码在`2026.10.11`分支，固定发布标签为`field-2026.10.11`。恢复时使用该标签，避免分支后续更新改变参数。源码Git树只保存源码、配置、脚本、文档和依赖清单；运行资源和原始日志作为同一个Release的独立附件。

## 快速恢复

前提是同一套步兵硬件和Ubuntu 22.04 x86_64桌面系统，用户名`rm`、家目录`/home/rm`，网络能访问GitHub。若整个系统被清空，先安装Ubuntu和`git`、`python3`；磁盘至少预留4GiB。改变相机、镜头、安装位置或下位机固件后，需要重新核对相应标定和协议。

```bash
sudo apt-get update
sudo apt-get install -y git python3 usbutils zenity libnotify-bin
git clone --branch field-2026.10.11 https://github.com/Reimuaa/Fujian_AutoAim.git /home/rm/sp_vision
cd /home/rm/sp_vision

# 下载附件并检查整包和每个文件的SHA-256；只检查，不安装、不启动。
python3 scripts/restore_runtime.py

# 校验通过后恢复运行文件和三个桌面入口；不启动自瞄、不烧录。
sudo python3 scripts/restore_runtime.py --apply
sudo usermod -aG dialout,plugdev rm
```

退出登录再登录，使串口组权限生效；重新插入相机使设备规则生效。恢复程序已安装现场原有的HikRobot `2bdf` USB规则并重新加载规则，不触发设备、不启动程序。连接原相机和下位机；恢复程序会提示缺失的共享库。

确认`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`存在、相机USB为`2bdf:0001`。由操作者使用桌面“启动自瞄”选择红/蓝；也可手动执行`bash tools/field_start_autoaim.sh --select-color`。GUI桌面首次可能需要右键“允许启动”。停止入口为`bash tools/field_stop_autoaim.sh`。恢复脚本本身不会启动这些入口。

可提前下载Release的`runtime.tar.gz`到外部磁盘；离线安装使用：

```bash
sudo python3 scripts/restore_runtime.py --bundle /外部磁盘/runtime.tar.gz --apply
```

附件名称、大小和SHA-256固定在`docs/recovery.json`。脚本会拒绝错误平台、变更过的正式配置、路径越界、压缩包内链接、重复条目和错误校验和。SDK恢复到原`/opt`路径；模型和库放回原位置；现场可执行文件在唯一`build/standard`，其他已链接系统库在`build/runtime/lib`。glibc和系统动态链接器由Ubuntu提供，不从现场覆盖。

## 保留的现场参数

| 项目 | 数值 |
|---|---|
| Aimer高/低速预测延迟 | 325ms / 325ms |
| 独立高速事件发射延迟 | 85ms，配置区间65—105ms；不代表实测离膛 |
| 旋转预测水平阈值 | 0.5° |
| 配置弹速 | 20m/s，未独立测速 |
| P轴弹道微调 | -0.50° |
| ROI | 开，420/300/600/600 |
| 角点细化 | 开 |
| 详细诊断、detector_debug、debug | 关 |
| 开火命令高/低电平保持 | 450ms / 550ms |
| 额外重装填间隔 | 0ms |
| IMU时间对齐 | +15ms |

当前程序SHA-256为`ece053d6f494d72e0a0d923aec38af1767eea5099a1da3f16e59d191965cff90`。恢复包保存的是现场已运行的这个程序，而不是把新编译产物冒充现场验收版本。

发布前验证：整包及27440个运行文件校验通过；在系统临时目录模拟恢复后，现场程序的全部非glibc链接库均解析到恢复出的文件，恢复出的OpenVINO成功在CPU编译恢复出的YOLO26模型。5项恢复安全测试、24项诊断回归、Linux库编译、旋转模型/ESKF离线测试和不执行程序的链接检查通过。模拟恢复结束后已清理临时运行文件，没有执行standard或打开相机/串口。未进行整机重装后的实弹复测。

## 重新编译

快速恢复可直接使用现场程序，不要求先安装编译器。需要修改源码时，先安装常用编译依赖并获取固定的子模块：

```bash
sudo apt-get install -y build-essential cmake libopencv-dev libeigen3-dev \
  libceres-dev libfmt-dev libspdlog-dev libyaml-cpp-dev nlohmann-json3-dev \
  libusb-1.0-0-dev python3-yaml
git submodule update --init --recursive
python3 scripts/check_runtime_assets.py
bash scripts/build.sh standard
```

完整现场依赖版本和三个子模块提交在`docs/runtime-environment.json`。编译仍只使用`build/`。先由用户停止现有程序，再更换或重新编译可执行文件；编译不会自动启动自瞄。Linux构建使用目标分支中同时兼容Windows的构建脚本，相关差异只用于平台适配；现场算法与正式配置按哈希核对。

## 数据和限制

`field-logs.tar.gz`保留2026-10-11的程序日志及仍存在的当天启动日志；它不包含录像、个人目录、SSH密钥或账号凭据。压缩包内`manifest.json`记录每份原始文件的大小和SHA-256。将其中`logs/`解压到正式工程即可离线分析，不需要启动硬件。各轮配置不同，不能把全天数据都当作325ms、-0.50°的验收数据。

用户报告此版本准确率很高；没有完整的实际发弹数、命中数和离膛时间，因此没有统计命中率百分比。此前旋转时投影和云台漂移的根因也没有在本轮被证明或修改。此次保留的是认可的运行版本、参数和原始证据。

下位机实际烧录版本仍未确认。恢复包不读取或覆盖Flash；若只清空小电脑，下位机原固件继续保留。若下位机也被清空，不能把仓库固件源码未经验证就当成现场相同固件刷入。不同操作系统、硬件安装或固件的恢复不能仅靠复制参数保证相同效果。
