#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import json
import threading
import time
from typing import Dict, Any
HOST_PC = "192.168.123.15"
HOST_PI = "192.168.123.161"
PORT_TO_PI =  52013
PORT_FROM_13 =  52014
PORT_FROM_15 =  12345


class Sender:
    def __init__(self, host: str, port: int):
        # 初始化网络参数
        self.send_skt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_edp = (host, port)
        
        # 初始化数据模板
        self.send_map: Dict[str, Any] = {
            "v_x": 0.5, "v_y": 0, "yaw_speed": 0,
            "move_mode": 0, "gait_type": 2,
            "arm_mode": 0, "body_height": 0,
            "light_mode": 0, "finish": 0
        }
        
        # 线程控制
        self.running = False
        self.send_mtx = threading.Lock()
        self.send_thread = None

        # 打开socket
        try:
            self.send_skt.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except socket.error as ec:
            print(f"ER->Sender: Fail to open socket: {ec}")

    def send_mes_th(self):
        while self.running:
            # 构建JSON数据
            with self.send_mtx:
                message = json.dumps(self.send_map).encode('utf-8')
            
            # 发送数据
            try:
                self.send_skt.sendto(message, self.send_edp)
            except socket.error as ec:
                print(f"Error sending message: {ec}")
            
            # 保持40ms间隔
            time.sleep(0.04)

    def stop(self):
        self.running = False
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join()

    def start(self):
        if not self.send_thread or not self.send_thread.is_alive():
            self.running = True
            self.send_thread = threading.Thread(target=self.send_mes_th, daemon=True)
            self.send_thread.start()
#sender_pi = Sender(HOST_PI, PORT_TO_PI)

# 使用示例（保持与C++相同的调用方式）
if __name__ == "__main__":
    sender_pi = Sender("192.168.123.161", 52013)
    while True:
      sender_pi.start()
      #time.sleep(2)
        #sender_pi.stop()