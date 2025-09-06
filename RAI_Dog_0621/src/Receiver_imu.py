#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import threading
import socket
import json
from typing import Dict, Any
HOST_PC = "192.168.123.15"
HOST_PI = "192.168.123.161"
PORT_TO_PI =  52013
PORT_FROM_13 =  52014
PORT_FROM_15 =  12345
PORT_15 = 23456

yaw = 0.0
az = 0.0
left_distance = 0
pitch = 0.0

class Receiver_Imu:
    def __init__(self, host: str, port: int):
        # 网络参数初始化
        self.recv_skt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = host
        self.port = port
        
        # 线程控制
        self.running = False
        self.recv_thread = None
        self.yaw = 0.0
        self.pitch = 0.0
        self.az = 0.0
        self.left_distance = 0
        self.right_distance = 0
        
        
        try:
            self.recv_skt.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.recv_skt.bind((self.host, self.port))
            self.recv_skt.settimeout(1)  # 设置接收超时
        except socket.error as e:
            print(f"ER->Receiver: Socket init failed: {e}")
            raise

    def recv_mes_th(self):
        while self.running:
            try:
                # 接收原始数据
                data, addr = self.recv_skt.recvfrom(1024)
                #print(f"data:{data}")         
                # 解析JSON数据
                try:
                    recv_map = json.loads(data.decode('utf-8'))
                    self.handle_data(recv_map, addr)
                except json.JSONDecodeError:
                    print(f"Malformed JSON from {addr}")            
            except socket.timeout:
                continue  # 正常超时继续循环
            except Exception as e:
                print(f"Unexpected error: {e}")
                break

    def handle_data(self, data: Dict[str, Any], addr: tuple):
        #global yaw,az,left_distance,pitch
        self.yaw = data["yaw"]
        self.pitch = data["pitch"]
        self.az = data["az"]
        self.left_distance = data["left_distance"]
        self.right_distance = data["right_distance"]
       # self.front_distance = data["front_distance"]
        
        #print(f"imu_yaw:{self.yaw}")
        #print(f"imu_az:{self.az}")
        #print(f"imu_pitch:{self.pitch}")
        
       # print(yaw)

    def start(self):
 
        if not self.recv_thread or not self.recv_thread.is_alive():
            self.running = True
            self.recv_thread = threading.Thread(target=self.recv_mes_th, daemon=True)
            self.recv_thread.start()
            print(f"Receiver started on {self.host}:{self.port}")

    def stop(self):
        self.running = False
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=2)
        self.recv_skt.close()
        print(f"Packets: {self.packet_count}, Errors: {self.error_count}")

if __name__ == '__main__':
    receiver_imu = Receiver_Imu(HOST_PC,PORT_15)
    receiver_imu.start()
    while True:
        #time.sleep(0.1)
        #print(f"imu_yaw:{receiver_imu.yaw}")
        #print(f"imu_az:{receiver_imu.az}")
        #print(f"imu_pitch:{receiver_imu.pitch}")
       # print(f"left_distance:{receiver_imu.left_distance} mm")
        print(f"right_distance:{receiver_imu.right_distance} mm")

