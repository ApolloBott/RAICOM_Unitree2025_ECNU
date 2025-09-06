#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检查和测试串口设备
"""

import os
import sys
import serial
import time
from serial.tools import list_ports

def print_header(title):
    """打印带装饰的标题"""
    print("\n" + "="*60)
    print(" " + title)
    print("="*60)

def list_all_serial_devices():
    """列出所有可用的串口设备"""
    print_header("系统中所有的串口设备")
    
    # 使用pyserial的list_ports功能
    ports = list(list_ports.comports())
    if not ports:
        print("没有找到任何串口设备!")
        return
        
    print(f"找到 {len(ports)} 个串口设备:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.device} - {port.description}")
        print(f"   硬件ID: {port.hwid}")
        print(f"   制造商: {port.manufacturer if hasattr(port, 'manufacturer') else '未知'}")
        print(f"   序列号: {port.serial_number if hasattr(port, 'serial_number') else '未知'}")
        print(f"   VID:PID = {port.vid:04x}:{port.pid:04x} 如果可用")
        print()
        
def check_symlinks():
    """检查/dev/目录下的相关符号链接"""
    print_header("检查设备符号链接")
    
    symlinks = {
        "detector0": "/dev/detector0",
        "detector1": "/dev/detector1",
        "detector2": "/dev/detector2",
        "right": "/dev/right",
        "arm": "/dev/arm"
    }
    
    for name, path in symlinks.items():
        if os.path.exists(path):
            if os.path.islink(path):
                target = os.readlink(path)
                abs_target = os.path.join(os.path.dirname(path), target) if not target.startswith('/') else target
                if os.path.exists(abs_target):
                    print(f"{path} -> {target} [存在]")
                else:
                    print(f"{path} -> {target} [目标不存在!]")
            else:
                print(f"{path} 不是符号链接")
        else:
            print(f"{path} 不存在")
            
def try_open_port(port_name, baudrates=[9600, 115200, 921600]):
    """尝试用多个波特率打开一个串口"""
    print(f"\n尝试打开 {port_name}:")
    
    for baudrate in baudrates:
        try:
            print(f"  尝试波特率 {baudrate} bps... ", end='')
            ser = serial.Serial(port_name, baudrate, timeout=0.5)
            print("成功!")
            
            # 尝试读取一些数据
            try:
                ser.setDTR(True)
                ser.setRTS(True)
            except:
                pass
                
            print(f"  读取数据 (最多5次)... ")
            for i in range(5):
                try:
                    start = time.time()
                    data = ser.read(16)
                    elapsed = time.time() - start
                    
                    if data:
                        print(f"    读取 #{i+1}: 成功, {len(data)} 字节, 用时 {elapsed:.3f}秒")
                        print(f"      十六进制: {' '.join([f'{b:02x}' for b in data])}")
                    else:
                        print(f"    读取 #{i+1}: 无数据, 超时 {elapsed:.3f}秒")
                except Exception as e:
                    print(f"    读取 #{i+1}: 错误 - {e}")
                time.sleep(0.5)
                
            ser.close()
            print(f"  关闭 {port_name}")
            
        except Exception as e:
            print(f"失败: {e}")
    
def main():
    """主函数"""
    print("===== 串口设备诊断工具 =====")
    print(f"当前时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    list_all_serial_devices()
    check_symlinks()
    
    # 尝试打开每个设备
    print_header("测试设备通信")
    devices = ["/dev/detector0", "/dev/detector1", "/dev/detector2", "/dev/right", "/dev/arm"]
    
    for dev in devices:
        if os.path.exists(dev):
            try_open_port(dev)
        else:
            print(f"\n{dev} 不存在，跳过测试")
            
    print("\n诊断完成！")
    
if __name__ == "__main__":
    main()
