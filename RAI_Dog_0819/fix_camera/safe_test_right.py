#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
安全的Right传感器测试 - 使用上下文管理器确保串口正确关闭
"""

import time
import serial
import contextlib
from distance import read_distance1, get_last_error, clear_port_cache

class SafeSerial:
    """安全的串口管理器，确保正确关闭"""
    
    def __init__(self, port, baudrate=115200, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
    
    def __enter__(self):
        try:
            # 先尝试清理可能存在的连接
            clear_port_cache()
            time.sleep(0.1)
            
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"✓ 成功打开串口: {self.port}")
            return self.ser
        except Exception as e:
            print(f"✗ 打开串口失败: {self.port} - {e}")
            raise
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.ser:
            try:
                # 强制清空缓冲区
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
                # 重置控制线
                self.ser.setDTR(False)
                self.ser.setRTS(False)
                time.sleep(0.05)
                
                # 关闭串口
                self.ser.close()
                print(f"✓ 成功关闭串口: {self.port}")
                
                # 额外延迟确保资源释放
                time.sleep(0.1)
                
            except Exception as e:
                print(f"⚠ 关闭串口时出现异常: {self.port} - {e}")
            finally:
                self.ser = None

def test_safe_right_sensor():
    """使用安全方式测试right传感器"""
    print("=== 安全的Right传感器测试 ===")
    
    device_path = '/dev/right'
    
    # 测试1: 第一次打开
    print("\n--- 测试1: 第一次打开 ---")
    try:
        with SafeSerial(device_path) as ser:
            success_count = 0
            for i in range(5):
                distance = read_distance1(ser)
                if distance is not None:
                    success_count += 1
                    print(f"  读取 #{i+1}: {distance}mm")
                else:
                    error = get_last_error(ser.port)
                    print(f"  读取 #{i+1}: 失败 - {error}")
                time.sleep(0.1)
            print(f"第一次测试成功率: {success_count}/5")
    except Exception as e:
        print(f"第一次测试异常: {e}")
        return False
    
    # 间隔
    print("\n等待串口完全释放...")
    time.sleep(1)
    
    # 测试2: 第二次打开（这里之前会卡死）
    print("\n--- 测试2: 第二次打开 ---")
    try:
        with SafeSerial(device_path) as ser:
            success_count = 0
            for i in range(5):
                distance = read_distance1(ser)
                if distance is not None:
                    success_count += 1
                    print(f"  读取 #{i+1}: {distance}mm")
                else:
                    error = get_last_error(ser.port)
                    print(f"  读取 #{i+1}: 失败 - {error}")
                time.sleep(0.1)
            print(f"第二次测试成功率: {success_count}/5")
            
            if success_count > 0:
                print("✓ 第二次打开成功，卡死问题已修复！")
                return True
            else:
                print("✗ 第二次打开失败，仍有问题")
                return False
                
    except Exception as e:
        print(f"第二次测试异常: {e}")
        print("✗ 第二次打开时出现异常，卡死问题仍存在")
        return False

def test_multiple_cycles():
    """测试多个开关周期"""
    print("\n=== 多周期测试 ===")
    
    device_path = '/dev/right'
    success_cycles = 0
    
    for cycle in range(3):
        print(f"\n--- 周期 {cycle + 1} ---")
        try:
            with SafeSerial(device_path) as ser:
                distance = read_distance1(ser)
                if distance is not None:
                    success_cycles += 1
                    print(f"  周期 {cycle + 1}: 成功 - {distance}mm")
                else:
                    error = get_last_error(ser.port)
                    print(f"  周期 {cycle + 1}: 失败 - {error}")
        except Exception as e:
            print(f"  周期 {cycle + 1}: 异常 - {e}")
        
        time.sleep(0.5)  # 周期间隔
    
    print(f"\n多周期测试结果: {success_cycles}/3 成功")
    if success_cycles >= 2:
        print("✓ 多周期测试通过")
        return True
    else:
        print("✗ 多周期测试失败")
        return False

def test_rapid_open_close():
    """测试快速开关"""
    print("\n=== 快速开关测试 ===")
    
    device_path = '/dev/right'
    success_count = 0
    
    for i in range(5):
        try:
            print(f"  快速测试 #{i+1}...")
            with SafeSerial(device_path) as ser:
                distance = read_distance1(ser)
                if distance is not None:
                    success_count += 1
                    print(f"    成功: {distance}mm")
                else:
                    print(f"    失败: {get_last_error(ser.port)}")
            time.sleep(0.2)  # 短间隔
        except Exception as e:
            print(f"    异常: {e}")
    
    print(f"\n快速开关测试结果: {success_count}/5 成功")
    if success_count >= 4:
        print("✓ 快速开关测试通过")
        return True
    else:
        print("✗ 快速开关测试失败")
        return False

def comprehensive_safe_test():
    """综合安全测试"""
    print("=== Right传感器综合安全测试 ===")
    
    # 首先强制清理
    print("清理所有缓存和资源...")
    clear_port_cache()
    time.sleep(0.5)
    
    results = []
    
    # 测试1: 基本安全测试
    results.append(test_safe_right_sensor())
    
    # 测试2: 多周期测试
    results.append(test_multiple_cycles())
    
    # 测试3: 快速开关测试
    results.append(test_rapid_open_close())
    
    # 总结
    success_count = sum(results)
    print(f"\n" + "="*50)
    print(f"测试总结: {success_count}/3 项测试通过")
    
    if success_count == 3:
        print("✓ 所有测试通过！Right传感器卡死问题已完全解决！")
        print("可以安全使用 Run_crossing1.py 和 Run_crossing2.py")
    elif success_count >= 2:
        print("⚠ 大部分测试通过，问题基本解决但可能还需要细微调整")
    else:
        print("✗ 多项测试失败，问题仍需进一步解决")
    
    return success_count == 3

if __name__ == "__main__":
    comprehensive_safe_test()
