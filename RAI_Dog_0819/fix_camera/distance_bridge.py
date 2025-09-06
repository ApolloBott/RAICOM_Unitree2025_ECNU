#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
这个桥接模块用于无缝集成旧的distance_old.py和新的distance.py
它提供了兼容的API，同时使用了更健壮的实现
"""

import serial
import time
import sys

# 尝试导入，失败时提供更有帮助的错误信息
try:
    # 导入新的实现
    from distance import read_distance1 as new_read_distance1
    from distance import read_distance as new_read_distance
    print("成功导入distance模块的函数")
except ImportError as e:
    print(f"警告: 无法导入distance模块: {e}")
    print("将使用本地实现的读取函数")
    
    # 定义本地实现的读取函数
    def new_read_distance1(serial_port):
        """本地实现的read_distance1函数"""
        # 检查是否有足够数据可读
        if serial_port.in_waiting < 8:
            return None
            
        # 读取数据
        frame = serial_port.read(8)
        if len(frame) < 5:  # 至少需要5字节才能解析
            return None
            
        # 验证帧头并解析
        if frame[0] == 0x01 and frame[1] == 0x03 and frame[2] == 0x02:
            distance = (frame[3] << 8) | frame[4]
            if distance <= 30000:
                return distance
        return None
        
    def new_read_distance(serial_port):
        """本地实现的read_distance函数"""
        # 检查是否有足够数据可读
        if serial_port.in_waiting < 16:
            return None
            
        # 读取数据
        frame = serial_port.read(16)
        if len(frame) < 16:
            return None
            
        # 验证帧头并解析
        if frame[0] == 0x57:
            check_sum = sum(frame[:15]) & 0xFF
            if check_sum == frame[15] and frame[8] != 0:
                distance = (frame[10] << 16) | (frame[9] << 8) | frame[8]
                return distance
        return None

# 全局变量，兼容旧代码
Distance = 0
Distance1 = 0

def read_distance1(serial_port):
    """兼容旧的API，但使用新的实现"""
    global Distance1
    if serial_port is None or not serial_port.is_open:
        print("警告: 串口未打开或无效")
        return Distance1
        
    try:
        # 确保有数据可读
        start_time = time.time()
        while serial_port.in_waiting < 1 and time.time() - start_time < 0.5:
            time.sleep(0.01)
            
        if serial_port.in_waiting > 0:
            # 首先尝试发送ModBus请求
            try:
                # ModBus读取请求 - 站号1，功能码3，寄存器0x0010，1个寄存器
                cmd = bytearray([0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF])
                serial_port.write(cmd)
                serial_port.flush()
                time.sleep(0.1)
            except:
                pass
                
            # 现在尝试读取
            result = new_read_distance1(serial_port)
            if result is not None:
                Distance1 = result
                return Distance1
            else:
                # 清空缓冲区
                serial_port.reset_input_buffer()
        return Distance1
        
    except Exception as e:
        print("Error in read_distance1: {}".format(e))
        # 清空缓冲区
        try:
            serial_port.reset_input_buffer()
        except:
            pass
        return Distance1

def read_distance(ser):
    """兼容旧的API，但使用新的实现"""
    global Distance
    if ser is None or not ser.is_open:
        print("警告: 串口未打开或无效")
        return Distance
        
    try:
        # 尝试主动等待数据
        start_time = time.time()
        while ser.in_waiting < 1 and time.time() - start_time < 0.3:
            time.sleep(0.01)
            
        # 如果没有数据，可能需要发送命令触发传感器
        if ser.in_waiting == 0:
            try:
                # 对于某些传感器，发送任意数据可以触发自动上报
                ser.write(b'\xAA')
                ser.flush()
                time.sleep(0.1)
            except:
                pass
                
        # 重新检查是否有数据可读
        if ser.in_waiting > 0:
            result = new_read_distance(ser)
            if result is not None:
                Distance = result
                return Distance
            
            # 尝试使用read_distance1作为备选方案
            try:
                result = new_read_distance1(ser)
                if result is not None:
                    Distance = result
                    return Distance
            except:
                pass
                
            # 清空缓冲区
            ser.reset_input_buffer()
        return Distance
        
    except Exception as e:
        print("Error in read_distance: {}".format(e))
        # 清空缓冲区
        try:
            ser.reset_input_buffer()
        except:
            pass
        return Distance

def init_serial_port(port_name, baudrate=115200, timeout=1, retry=3):
    """初始化串口并尝试拉起DTR/RTS，带重试功能"""
    # 如果端口是符号链接，解析成实际设备
    real_device = port_name
    if os.path.islink(port_name):
        try:
            link_target = os.readlink(port_name)
            real_device = os.path.join(os.path.dirname(port_name), link_target) if not link_target.startswith('/') else link_target
            print("端口 {} 是符号链接，指向 {}".format(port_name, real_device))
        except:
            pass
    
    # 设置端口特定的波特率，根据实际情况调整
    port_baudrates = {
        "/dev/detector0": [115200, 9600],
        "/dev/detector1": [115200, 9600],
        "/dev/detector2": [115200, 9600],
        "/dev/right": [115200, 9600],
        "/dev/arm": [115200]
    }
    
    # 如果指定了设备特定的波特率，使用它们
    baudrates = port_baudrates.get(port_name, [baudrate])
    
    # 尝试各种波特率
    for baud in baudrates:
        for attempt in range(retry):
            try:
                print("尝试打开 {} (波特率={}, 尝试#{})...".format(port_name, baud, attempt+1))
                ser = serial.Serial(real_device, baud, timeout=timeout)
                
                # 清空缓冲区
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                
                # 尝试拉起DTR/RTS信号 
                try:
                    ser.setDTR(True)
                    ser.setRTS(True)
                except Exception as e:
                    print("警告: 无法设置DTR/RTS信号: {}".format(e))
                
                print("成功打开 {} @ {} bps".format(port_name, baud))
                # 延时，让设备准备好
                time.sleep(0.2)
                
                # 检测是否可以读取数据
                try:
                    # ModBus读取请求 - 站号1，功能码3，寄存器0x0010，1个寄存器
                    cmd = bytearray([0x01, 0x03, 0x00, 0x10, 0x00, 0x01, 0x85, 0xCF])
                    ser.write(cmd)
                    ser.flush()
                    time.sleep(0.1)
                    
                    if ser.in_waiting > 0:
                        print("检测到 {} 的响应".format(port_name))
                except:
                    pass
                    
                return ser
                
            except Exception as e:
                print("尝试 #{} 失败: {}".format(attempt+1, e))
                if attempt < retry - 1:
                    print("将在1秒后重试...")
                    time.sleep(1)
    
    print("错误: 无法初始化 {} 在 {} 次尝试后".format(port_name, retry))
    return None

def send_init_cmd(ser, cmd_bytes=None):
    """向传感器发送初始化命令，可以帮助某些传感器正确响应"""
    if ser is None or not ser.is_open:
        return False
    
    # 如果没有提供命令，使用默认的ModBus读取命令(读取寄存器0x10)
    if cmd_bytes is None:
        # ModBus读取命令: 地址1, 功能码3, 寄存器0x0010, 读取1个寄存器
        cmd_bytes = b'\x01\x03\x00\x10\x00\x01\x85\xcf'
    
    try:
        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # 发送命令
        ser.write(cmd_bytes)
        ser.flush()
        
        # 等待响应
        time.sleep(0.1)
        
        # 检查是否有响应
        return ser.in_waiting > 0
    except:
        return False

if __name__ == "__main__":
    # 与distance_old.py相同的测试代码
    print("=== 测距传感器测试 ===")
    
    # 使用不同的端口地址，避免冲突
    ports = [
        # (端口名称, 波特率, 读取函数, 描述)
        ('/dev/ttyCH343USB2', 115200, read_distance1, "传感器1(detector0)"),
        ('/dev/ttyCH343USB1', 115200, read_distance1, "传感器2(detector1)"),
        ('/dev/ttyCH343USB3', 115200, read_distance1, "传感器3(right)"),
    ]
    
    # 打开所有端口
    serial_ports = []
    for port_name, baudrate, _, desc in ports:
        ser = init_serial_port(port_name, baudrate)
        if ser:
            # 尝试发送初始化命令
            if send_init_cmd(ser):
                print(f"成功初始化 {desc} ({port_name})")
            else:
                print(f"初始化 {desc} ({port_name}) 未检测到响应")
        serial_ports.append(ser)
    
    # 检查是否所有端口都打开成功
    if None in serial_ports:
        print("警告: 部分串口无法打开")
    
    # 开始测试读取
    print("\n开始读取测距数据 (按Ctrl+C退出)...")
    try:
        while True:
            print("\n===== 测距结果 =====")
            for i, (port_name, _, read_func, desc) in enumerate(ports):
                if serial_ports[i]:
                    distance = read_func(serial_ports[i])
                    status = "有效" if distance > 0 else "无效"
                    print(f"{desc}: {distance} mm ({status})")
                else:
                    print(f"{desc}: 串口未打开")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n测试终止")
    finally:
        # 关闭所有端口
        for ser in serial_ports:
            if ser and ser.is_open:
                ser.close()
