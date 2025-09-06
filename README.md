# RAICOM_Unitree2025
[![Platform](https://img.shields.io/badge/Platform-Linux-orange)]()
[![Python](https://img.shields.io/badge/Python-3.10--3.12-blue)]()
[![License](https://img.shields.io/badge/License-MIT-lightgrey)]()
## 💡Introduction
2025RAICOM四组多模态赛道 **RAICOM_Unitree2025**, 项目采用Unitree Go2，幻尔机械臂

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

## 🚀 Quick start

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

### 3. 运行实例
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
