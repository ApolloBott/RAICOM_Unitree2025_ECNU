#!/usr/bin/env python3
# 桥接文件，将原始ArmControl替换为优化版本

import sys
import os

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 从优化版本导入所有内容
from ArmControl_optimized import *

# 在这里可以添加兼容层代码，如果有必要的话
print("已加载优化版ArmControl")

if __name__ == "__main__":
    print("ArmControl桥接模块 - 提供原始API与优化实现之间的兼容层")
    print("此模块不应直接运行，而应被其他程序导入")
