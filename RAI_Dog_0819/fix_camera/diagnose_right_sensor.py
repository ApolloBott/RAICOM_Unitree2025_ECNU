#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Right传感器专项诊断工具
"""

import serial
import time
from distance import _mb_read_u16, _mb_req_read, _crc16_modbus

def diagnose_right_sensor_detailed():
    """详细诊断right传感器"""
    print("=== Right传感器详细诊断 ===")
    
    device_path = '/dev/right'
    
    try:
        ser = serial.Serial(device_path, 115200, timeout=1.0)  # 增加超时时间
        print(f"✓ 串口打开成功: {device_path}")
        
        # 设置控制信号
        ser.setDTR(True)
        ser.setRTS(True)
        ser.reset_input_buffer()
        
        # 测试不同的波特率和从站地址组合
        baud_rates = [115200, 9600, 19200, 38400, 57600]
        slave_addrs = [1, 2, 3, 4, 5, 10]
        
        print("\n详细测试不同配置组合:")
        
        for baud in baud_rates:
            print(f"\n测试波特率: {baud}")
            try:
                ser.baudrate = baud
                ser.reset_input_buffer()
                time.sleep(0.1)  # 给设备时间适应新波特率
                
                for slave in slave_addrs:
                    print(f"  测试从站地址: {slave}")
                    
                    # 发送ModBus请求
                    req = bytes([slave, 0x03, 0x00, 0x10, 0x00, 0x01])
                    crc = _crc16_modbus(req)
                    req += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
                    
                    try:
                        ser.write(req)
                        ser.flush()
                        
                        # 等待响应
                        time.sleep(0.1)
                        
                        if ser.in_waiting > 0:
                            response = ser.read(ser.in_waiting)
                            print(f"    响应: {response.hex()}")
                            
                            if len(response) >= 5:
                                if response[0] == slave and response[1] == 0x03:
                                    distance = (response[3] << 8) | response[4]
                                    print(f"    ✓ 成功! 距离: {distance}mm")
                                    ser.close()
                                    return True
                                else:
                                    print(f"    响应格式不正确")
                            else:
                                print(f"    响应长度不足: {len(response)}字节")
                        else:
                            print(f"    无响应")
                        
                        ser.reset_input_buffer()
                        
                    except Exception as e:
                        print(f"    通信异常: {e}")
            
            except Exception as e:
                print(f"  设置波特率{baud}失败: {e}")
        
        # 尝试检测设备是否发送自动数据
        print(f"\n检测设备是否有自动发送的数据...")
        ser.baudrate = 115200
        ser.reset_input_buffer()
        
        for i in range(10):
            time.sleep(0.5)
            if ser.in_waiting > 0:
                auto_data = ser.read(ser.in_waiting)
                print(f"自动数据 #{i+1}: {auto_data.hex()}")
        
        ser.close()
        print("✗ 所有配置都无法通信")
        return False
        
    except Exception as e:
        print(f"✗ 串口连接失败: {e}")
        return False

def test_right_sensor_alternatives():
    """测试right传感器的其他可能协议"""
    print("\n=== 测试其他可能的协议 ===")
    
    device_path = '/dev/right'
    
    protocols_to_test = [
        {
            'name': 'ASCII协议',
            'commands': [b'R\r\n', b'READ\r\n', b'DIST\r\n', b'?\r\n'],
            'baud': 9600
        },
        {
            'name': 'HEX命令协议',
            'commands': [b'\x55\xAA\x01\x00\x01', b'\xFF\x01\x86', b'\x01\x03\x00\x00\x00\x01'],
            'baud': 115200
        }
    ]
    
    try:
        ser = serial.Serial(device_path, 115200, timeout=1.0)
        
        for protocol in protocols_to_test:
            print(f"\n测试 {protocol['name']}:")
            ser.baudrate = protocol['baud']
            ser.reset_input_buffer()
            time.sleep(0.1)
            
            for i, cmd in enumerate(protocol['commands']):
                print(f"  发送命令 #{i+1}: {cmd}")
                try:
                    ser.write(cmd)
                    ser.flush()
                    time.sleep(0.2)
                    
                    if ser.in_waiting > 0:
                        response = ser.read(ser.in_waiting)
                        print(f"    响应: {response.hex()} (ASCII: {response})")
                    else:
                        print(f"    无响应")
                except Exception as e:
                    print(f"    发送失败: {e}")
        
        ser.close()
        
    except Exception as e:
        print(f"协议测试失败: {e}")

if __name__ == "__main__":
    success = diagnose_right_sensor_detailed()
    
    if not success:
        test_right_sensor_alternatives()
        
        print("\n=== 诊断结论 ===")
        print("Right传感器可能的问题:")
        print("1. 设备使用非标准ModBus协议")
        print("2. 设备可能损坏或供电不足")
        print("3. 设备可能是不同型号的传感器")
        print("4. 需要特殊的初始化序列")
        print("\n建议:")
        print("- 检查设备型号和数据手册")
        print("- 确认设备供电电压")
        print("- 检查接线是否正确")
        print("- 尝试使用厂商提供的测试软件")
