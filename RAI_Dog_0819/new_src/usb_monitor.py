#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB串口设备监控和诊断工具
"""

import os
import time
import subprocess
import re

class USBSerialMonitor:
    """USB串口设备监控器"""
    
    def __init__(self):
        self.device_paths = ['/dev/arm', '/dev/detector0', '/dev/detector1', '/dev/detector2', '/dev/right']
        self.last_check_time = 0
        
    def check_usb_devices(self):
        """检查USB设备状态"""
        print("=== USB串口设备状态检查 ===")
        
        # 检查设备文件是否存在
        existing_devices = []
        missing_devices = []
        
        for device in self.device_paths:
            if os.path.exists(device):
                existing_devices.append(device)
                print(f"✓ {device}: 设备文件存在")
            else:
                missing_devices.append(device)
                print(f"✗ {device}: 设备文件不存在")
        
        return existing_devices, missing_devices
    
    def get_usb_device_info(self):
        """获取USB设备详细信息"""
        try:
            # 获取USB设备列表
            result = subprocess.run(['lsusb'], capture_output=True, text=True)
            usb_devices = result.stdout
            
            print("\n=== 连接的USB设备 ===")
            print(usb_devices)
            
            # 查找CH341芯片设备
            ch341_devices = []
            for line in usb_devices.split('\n'):
                if 'ch340' in line.lower() or 'ch341' in line.lower():
                    ch341_devices.append(line.strip())
            
            if ch341_devices:
                print("\n=== 检测到的CH341设备 ===")
                for device in ch341_devices:
                    print(f"  {device}")
            else:
                print("\n[WARNING] 未检测到CH341设备")
                
        except Exception as e:
            print(f"[ERROR] 获取USB设备信息失败: {e}")
    
    def check_dmesg_logs(self, lines=50):
        """检查系统日志中的USB相关信息"""
        try:
            result = subprocess.run(['dmesg', '--time-format=iso'], capture_output=True, text=True)
            logs = result.stdout.split('\n')
            
            # 筛选USB和串口相关日志
            usb_logs = []
            keywords = ['usb', 'ttyUSB', 'ch341', 'uart', 'serial']
            
            for log in logs[-lines:]:
                if any(keyword in log.lower() for keyword in keywords):
                    usb_logs.append(log)
            
            if usb_logs:
                print(f"\n=== 最近{lines}行中的USB/串口相关日志 ===")
                for log in usb_logs[-20:]:  # 显示最近20条
                    print(log)
            
            # 查找错误信息
            error_patterns = [
                r'urb stopped: -32',
                r'device disconnected',
                r'I/O error',
                r'converter now disconnected',
                r'converter now attached'
            ]
            
            recent_errors = []
            for log in logs[-100:]:  # 检查最近100行
                for pattern in error_patterns:
                    if re.search(pattern, log, re.IGNORECASE):
                        recent_errors.append(log)
            
            if recent_errors:
                print(f"\n=== 检测到的USB错误信息 ===")
                for error in recent_errors[-10:]:  # 显示最近10个错误
                    print(f"  {error}")
                    
        except Exception as e:
            print(f"[ERROR] 检查系统日志失败: {e}")
    
    def diagnose_connection_issues(self):
        """诊断连接问题"""
        print("\n=== 连接问题诊断 ===")
        
        # 1. 检查USB端口状态
        try:
            result = subprocess.run(['ls', '-la', '/sys/bus/usb/devices/'], capture_output=True, text=True)
            usb_ports = result.stdout
            print("USB端口状态:")
            for line in usb_ports.split('\n'):
                if 'usb' in line and '->' in line:
                    print(f"  {line.strip()}")
        except:
            pass
        
        # 2. 检查串口设备权限
        print("\n串口设备权限:")
        for device in self.device_paths:
            if os.path.exists(device):
                stat = os.stat(device)
                print(f"  {device}: 权限 {oct(stat.st_mode)[-3:]}")
        
        # 3. 检查进程占用
        try:
            result = subprocess.run(['lsof'] + [d for d in self.device_paths if os.path.exists(d)], 
                                  capture_output=True, text=True, timeout=5)
            if result.stdout.strip():
                print(f"\n设备占用情况:")
                print(result.stdout)
            else:
                print(f"\n设备未被其他进程占用")
        except:
            print(f"\n无法检查设备占用情况")
    
    def monitor_realtime(self, duration=30):
        """实时监控USB设备状态"""
        print(f"\n=== 开始实时监控 {duration} 秒 ===")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            existing, missing = self.check_usb_devices()
            
            if missing:
                print(f"[{time.strftime('%H:%M:%S')}] 检测到设备断开: {missing}")
            
            time.sleep(2)
        
        print("实时监控结束")

def main():
    monitor = USBSerialMonitor()
    
    print("USB串口设备监控和诊断工具")
    print("=" * 50)
    
    # 基本检查
    monitor.check_usb_devices()
    monitor.get_usb_device_info()
    monitor.check_dmesg_logs()
    monitor.diagnose_connection_issues()
    
    # 提供建议
    print("\n=== 问题解决建议 ===")
    print("1. 硬件检查:")
    print("   - 检查USB线缆连接是否牢固")
    print("   - 尝试更换USB端口")
    print("   - 检查USB线缆质量")
    print("   - 确保USB供电充足")
    
    print("\n2. 软件解决:")
    print("   - 重新插拔USB设备")
    print("   - 重启USB子系统: sudo modprobe -r ch341 && sudo modprobe ch341")
    print("   - 禁用USB自动挂起: echo -1 > /sys/module/usbcore/parameters/autosuspend")
    
    print("\n3. 监控建议:")
    print("   - 运行此脚本进行实时监控")
    print("   - 观察系统日志: sudo dmesg -w | grep -i usb")
    
    # 询问是否进行实时监控
    try:
        response = input("\n是否进行30秒实时监控? (y/n): ")
        if response.lower() == 'y':
            monitor.monitor_realtime(30)
    except KeyboardInterrupt:
        print("\n监控已取消")

if __name__ == "__main__":
    main()
