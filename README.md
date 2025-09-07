# RAICOM_Unitree2025_ECNU
[![Platform](https://img.shields.io/badge/Platform-Linux-orange)]()
[![Python](https://img.shields.io/badge/Python-3.10--3.12-blue)]()
[![License](https://img.shields.io/badge/License-MIT-lightgrey)]()
## 💡Introduction
2025RAICOM四组多模态赛道 **RAICOM_Unitree2025**, 项目采用Unitree Go2、幻尔机械臂，适用于一切六轴机械臂。

> ⚠️ **注意**：本项目仅支持 **Linux (Ubuntu 20.04/22.04)** 环境，请不要在 Windows 下直接运行。

---

## ✨ Features
-  **多传感器融合**：IMU、测距传感器
-  **跨平台协同**：四足机器人 + 机械臂
-  **智能控制**：基于状态机与 PID 的精确控制
-  **鲁棒通信**：串口数据接收与错误恢复机制
-  **模块化设计**：视觉处理、外设控制、工具函数独立分层

---

## 📦 Project Structure
```text
RAI_Dog/
├── src/                            # 核心源代码目录
│   ├── 控制核心/
│   │   ├── Run_crossing*.py        # 主程序入口
│   │   └── Actions_crossing*.py    # 状态机控制
│   │
│   ├── 视觉处理/
│   │   └── Img_Processor.py        # 图像处理
│   │
│   ├── 外设控制/
│   │   ├── ArmControl.py           # 机械臂控制
│   │   ├── BusServoCmd.py          # 舵机命令接口
│   │   └── Board.py                # 主板硬件抽象层
│   │   └── distance.py             # 测距传感器接口
│   │
│   ├── 通信模块/
│   │   └── Receiver_imu.py         # IMU数据接收
│   │
│   ├── 工具模块/
│   │   ├── PID.py                  # PID控制器
│   │   └── Timer.py                # 定时器模块
│   │
│   └── 测试模块/
│       ├── prepare.sh              # 环境准备脚本
│       ├── Check_detector.py       # 串口检查
│       ├── ReadPosition.py         # 上位机机械臂控制
│       └── Readdetector.py         # 测距传感器检查
│
└── 测试图片/
    ├── 1.jpg ~ 6.jpg               # 测试图像
    └── tag1.jpg ~ tag3.jpg         # ArUco Tag 图像
```

---

## 🚀 Quick Start

### 1. 克隆仓库
```bash
git clone https://github.com/JOJOJANNIE/RAICOM_Unitree2025.git
cd RAICOM_Unitree2025
```
### 2. 一键安装依赖

- 本地 Python 版本为 3.12。
```powershell
pip install -r requirements.txt
```

- 💡 提示：如使用 Anaconda，也可手动创建环境：
```powershell
conda create -n raicom python=3.12
conda activate raicom
pip install -r requirements.txt
```

### 3. udev 规则配置
- 我们使用Orin Nano作为Unitree Go2的外接电脑。外接3个TOF200F激光测距模块，机械臂通过tty转USB连接到Orin。

#### 步骤1. 查询串口型号(将ttyCH343USB0替换为你需要查询的串口)：
```powershell
udevadm info -a -n /dev/ttyCH343USB0 | egrep -i "idVendor|idProduct|serial|KERNELS|product|manufacturer" -m 5
```
- 预计输出类似于：
```powershell
KERNELS=="1-2.2.1:1.0'
KERNELS=="1-2.2.1"
ATTRS{idProduct}=="55d3"
ATTRS{idVendor}=="1a86"
ATTRS{serial}=="5959048178'
```

#### 步骤2. 创建规则文件
```powershell
sudoedit /etc/udev/rules.d/99-x9x-serial.rules
```

#### 步骤3. 编辑udev规则
示例：
```powershell
KERNEL=="ttyCH343USB[0-9]*", MODE:="0660", GROUP:="dialout"
SUBSYSTEM=="tty", KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5959048178", SYMLINK+="detector0"
```
将具体的设备指纹换成**步骤1**中的输出，SYLINK是符号链接名，可以自行更改。


#### 步骤4. 保存并加载（记得可能要重新插拔串口）：
```powershell
sudo udevadm control --reload-rules && udevadm trigger
```

将用户添加到dialout组（可选）：
```powershell
sudo usermod -aG dialout 用户名
```

后期若要查找设备的符号链接，可以使用：
```powershell
find /dev -type l -lname "video*"
```
也可以直接使用`ls`，比如:
```powershell
ls -l /dev/arm
```

- 💡注意：
本项目中机械臂ttyUSB0 <-> arm，
机械臂6号舵机处测距ttyCH343USB0 <-> detector0，
机械臂2号舵机处测距ttyCH343USB1 <-> detector1，
机器狗右测距ttyCH343USB2 <-> right
- 具体测距传感器的分布及功能可以参考[省赛 / 国赛报告材料](./睿抗-物资运送-萝卜快跑-国赛-报告材料.pdf)


### 4. 运行实例
- 运行机械臂示例
```powershell
python RAI_Dog/src/ArmControl.py
```

- 运行完整项目
```powershell
python RAI_Dog/src/Run_crossing1.py
```

- 运行串口检查模块
```powershell
python RAI_Dog/src/Check_detector.py
```

## 📑 Reference

### 1. 赛项规则

完整的赛事规则请参考：[四组多模态巡检赛项规则文件](./四组多模态巡检赛项规则文件.pdf)

### 2. 报告材料

省赛/国赛报告材料请参考：[省赛报告材料](./睿抗-物资运送-萝卜快跑-省赛-报告材料.pdf) / [国赛报告材料](./睿抗-物资运送-萝卜快跑-国赛-报告材料.pdf)