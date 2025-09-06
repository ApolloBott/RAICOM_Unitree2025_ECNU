#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口工具模块，提供健壮的串口连接和错误恢复功能
"""

import serial
import time

class RobustSerialConnection:
    """健壮的串口连接类，提供自动重连和错误恢复功能"""
    
    def __init__(self, port_path, baudrate, timeout=1, max_retries=3):
        self.port_path = port_path
        self.baudrate = baudrate
        self.timeout = timeout
        self.max_retries = max_retries
        self.serial_obj = None
        self.is_connected = False
        self.reconnect_count = 0
        
    def connect(self):
        """建立串口连接"""
        try:
            if self.serial_obj and self.serial_obj.is_open:
                self.serial_obj.close()
                time.sleep(0.1)
            
            self.serial_obj = serial.Serial(
                self.port_path, 
                self.baudrate, 
                timeout=self.timeout
            )
            
            if self.serial_obj.is_open:
                self.is_connected = True
                print(f"[INFO] Successfully connected to {self.port_path}")
                return True
        except Exception as e:
            print(f"[ERROR] Failed to connect to {self.port_path}: {e}")
            self.is_connected = False
        return False
    
    def reconnect(self):
        """重新连接串口"""
        print(f"[INFO] Attempting to reconnect to {self.port_path}...")
        for attempt in range(self.max_retries):
            print(f"[INFO] Reconnection attempt {attempt + 1}/{self.max_retries}")
            if self.connect():
                self.reconnect_count += 1
                print(f"[INFO] Reconnection successful (total reconnects: {self.reconnect_count})")
                return True
            time.sleep(0.5)
        
        print(f"[ERROR] Failed to reconnect to {self.port_path} after {self.max_retries} attempts")
        self.is_connected = False
        return False
    
    def get_serial(self):
        """获取串口对象，如果连接断开会自动重连"""
        if not self.is_connected or not self.serial_obj or not self.serial_obj.is_open:
            if not self.reconnect():
                raise serial.SerialException(f"Unable to maintain connection to {self.port_path}")
        return self.serial_obj
    
    def safe_write(self, data):
        """安全的写入操作，包含重试机制"""
        for attempt in range(self.max_retries):
            try:
                serial_obj = self.get_serial()
                serial_obj.write(data)
                return True
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Write failed on attempt {attempt + 1}/{self.max_retries}: {e}")
                self.is_connected = False
                if attempt < self.max_retries - 1:
                    time.sleep(0.2)
        return False
    
    def safe_read(self, size=None):
        """安全的读取操作，包含重试机制"""
        for attempt in range(self.max_retries):
            try:
                serial_obj = self.get_serial()
                if size is None:
                    return serial_obj.read()
                else:
                    return serial_obj.read(size)
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Read failed on attempt {attempt + 1}/{self.max_retries}: {e}")
                self.is_connected = False
                if attempt < self.max_retries - 1:
                    time.sleep(0.2)
        return None
    
    def close(self):
        """关闭串口连接"""
        try:
            if self.serial_obj and self.serial_obj.is_open:
                self.serial_obj.close()
                print(f"[INFO] Closed connection to {self.port_path}")
        except Exception as e:
            print(f"[WARNING] Error closing {self.port_path}: {e}")
        finally:
            self.is_connected = False
    
    def __del__(self):
        """析构函数，确保串口被正确关闭"""
        self.close()

def create_robust_serial_connections():
    """创建所有需要的健壮串口连接"""
    connections = {}
    
    serial_configs = [
        ('ser0', '/dev/detector0', 921600),
        ('ser1', '/dev/detector1', 115200),
        ('ser2', '/dev/arm', 115200),
        ('ser3', '/dev/right', 115200),
    ]
    
    for name, port, baudrate in serial_configs:
        try:
            conn = RobustSerialConnection(port, baudrate, timeout=1)
            if conn.connect():
                connections[name] = conn
            else:
                print(f"[ERROR] Failed to establish initial connection to {name} ({port})")
                connections[name] = None
        except Exception as e:
            print(f"[ERROR] Exception while creating connection {name}: {e}")
            connections[name] = None
    
    return connections

def validate_serial_connections(connections):
    """验证所有串口连接"""
    failed_connections = []
    for name, conn in connections.items():
        if conn is None or not conn.is_connected:
            failed_connections.append(name)
    
    if failed_connections:
        print(f"[ERROR] Failed connections: {failed_connections}")
        return False
    
    print("[INFO] All serial connections validated successfully")
    return True
