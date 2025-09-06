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
│   │       ├── BusServoCmd.py      # 舵机命令接口 - 底层舵机控制
│   │       └── Board.py            # 主板控制接口 - 硬件抽象层
│   │   └── distance.py             # 测距传感器接口
│   │
│   ├── 通信模块/               
│   │   └── Receiver_imu.py         # IMU数据接收器
│   │
│   ├── 工具模块/              
│   │   ├── PID.py                  # PID控制器
│   │   └── Timer.py                # 定时器模块          
│   │
│   └── 测试模块/   
│       ├── prepare.sh              # 环境准备脚本
│       ├── Check_detector.py       # 各设备串口状态检查
│       ├── ReadPosition.py         # 机械臂上位机控制
│       └── Readdetector.py         # 测距传感器状态检查
│  
└── 测试图片/                       
    ├── 1.jpg ~ 6.jpg               # 测试图像样本
    └── tag1.jpg ~ tag3.jpg         # ArUco Tag测试图像                   