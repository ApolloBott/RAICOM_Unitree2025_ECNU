#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增强版的 Run_crossing1.py
"""

import os, sys, runpy
from Img_Processor import Img_Processor
from Actions_crossing1 import Actions
import cv2
import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PointStamped_ 
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_RANGE_INFO = "rt/utlidar/range_info"
import math
from distance import read_distance
import serial
from BusServoCmd import *
from ArmControl import ArmControl
from Timer import timer
from Board import setBusServoPulse
from SerialUtils import create_robust_serial_connections, validate_serial_connections

pitch = 0
front = 0
left = 0
right = 0

class HighStateHandler(ChannelSubscriber):
    def __init__(self):
        super().__init__(TOPIC_HIGHSTATE, SportModeState_)
        self.Init(self.on_data)
    def on_data(self, data):
        global pitch
        rpy = data.imu_state.rpy
        pitch = rpy[1]

class RobustArmControl(ArmControl):
    
    def __init__(self, serial_connections):
        super().__init__()
        self.serial_connections = serial_connections
    
    def get_robust_serial(self, serial_name):
        if serial_name in self.serial_connections and self.serial_connections[serial_name]:
            return self.serial_connections[serial_name].get_serial()
        else:
            raise serial.SerialException(f"Serial connection {serial_name} not available")
    
    def safe_arm_put_fixed(self):
        try:
            ser2 = self.get_robust_serial('ser2')
            self.arm_put_fixed(ser2)
        except Exception as e:
            print(f"[ERROR] arm_put_fixed failed: {e}")
            print("[INFO] Attempting to recover...")
            
            if 'ser2' in self.serial_connections and self.serial_connections['ser2']:
                if self.serial_connections['ser2'].reconnect():
                    print("[INFO] Serial connection restored, attempting basic reset...")
                    try:
                        ser2 = self.get_robust_serial('ser2')
                        self.reset(125, 850, 0, 100, 500, ser2)
                        print("[INFO] Basic reset completed")
                    except Exception as reset_error:
                        print(f"[WARNING] Basic reset also failed: {reset_error}")
            
            self.fin_crossing = True
            print("[INFO] Marked crossing as finished to continue execution")
    
    def safe_arm_catch_and_put(self):
        try:
            ser0 = self.get_robust_serial('ser0')
            ser1 = self.get_robust_serial('ser1') 
            ser2 = self.get_robust_serial('ser2')
            self.arm_catch_and_put(ser0, ser1, ser2)
        except Exception as e:
            print(f"[ERROR] arm_catch_and_put failed: {e}")
            print("[INFO] Attempting to recover...")
        
            self.fin_circle = True
            print("[INFO] Marked circle as finished to continue execution")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)
    ChannelFactoryInitialize(0, sys.argv[1])
    sport_client = SportClient()  
    sport_client.SetTimeout(10.0)
    sport_client.Init()
    handler1 = HighStateHandler()
    print("[INFO] Initializing robust serial connections...")
    serial_connections = create_robust_serial_connections()
    
    if not validate_serial_connections(serial_connections):
        print("[ERROR] Some serial connections failed. " \
        "The program will continue but some functions may not work properly.")
    armcontrol = RobustArmControl(serial_connections)
    

    img_processor = Img_Processor()
    camera_path = "/dev/video0"
    cap = cv2.VideoCapture(camera_path)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 440)   
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 440)  
    cap.set(cv2.CAP_PROP_FPS, 30) 
    actions = Actions()
    
    # 获取传统串口对象用于兼容性
    try:
        ser0 = serial_connections['ser0'].get_serial() if serial_connections['ser0'] else None
        ser1 = serial_connections['ser1'].get_serial() if serial_connections['ser1'] else None
        ser2 = serial_connections['ser2'].get_serial() if serial_connections['ser2'] else None
        ser3 = serial_connections['ser3'].get_serial() if serial_connections['ser3'] else None
    except Exception as e:
        print(f"[ERROR] Failed to get serial objects: {e}")
        print("[WARNING] Continuing with None serial objects - some functions may fail")
        ser0 = ser1 = ser2 = ser3 = None
    
    try:
        if ser2:
            armcontrol.reset(125, 850, 0, 150, 500, ser2)
            setBusServoPulse(4, 300, 1000, ser2)
            time.sleep(0.2)
            armcontrol.reset_arm(125, 850, 0, 150, 500, 0, ser2)
    except Exception as e:
        print(f"[WARNING] Failed to initialize arm position: {e}")
    
    actions.sm = "s_start"
    
    print("[INFO] Starting main loop...")
    
    try:
        while True:  
            if not img_processor.finish_visual:
                ret, frame = cap.read()
                if not ret:
                    print("Camera read failed, finishing")
                    break
                img_processor.image = frame
                img_processor.get_origin()
                img_processor.get_binary()
                img_processor.get_contours()
                img_processor.get_special_pnts()
                img_processor.get_evens()
                img_processor.get_width()
            
            print(f"s:{actions.micro_s}")
            print(f"sm:{actions.sm}")
            print(f"len:{len(img_processor.idss)}")
            print(f"idss:{img_processor.idss}")
            
            # 状态机执行，包含错误处理
            try:
                if actions.sm == 's_start':
                    actions.f_start(img_processor, sport_client, armcontrol, ser3)
                elif actions.sm == 's_in_circle':
                    actions.f_in_circle(img_processor, sport_client, armcontrol, ser0, ser1, ser2)
                elif actions.sm == 's_in_crossing1':
                    actions.f_in_crossing1(img_processor, sport_client, armcontrol, ser1, ser2)
                elif actions.sm == 's_in_crossing2':
                    actions.f_in_crossing2(img_processor, sport_client, armcontrol, ser1, ser2)
                elif actions.sm == 's_ign_crossing':
                    actions.f_ignore_crossing(img_processor, sport_client)
                elif actions.sm == 's_up':
                    actions.f_up(img_processor, sport_client, pitch)
                elif actions.sm == 's_finish':
                    actions.f_finish(img_processor, sport_client)
                elif actions.sm == 's_straight':
                    actions.f_go_straight(img_processor, sport_client)
            except Exception as e:
                print(f"[ERROR] Error in state {actions.sm}: {e}")
                print("[INFO] Continuing execution...")
            
            if cv2.waitKey(1) == 27:
                break
                
    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")
    except Exception as e:
        print(f"[ERROR] Unexpected error in main loop: {e}")
    finally:
        print("[INFO] Cleaning up...")
        try:
            for name, conn in serial_connections.items():
                if conn:
                    conn.close()
            
            cap.release()
            cv2.destroyAllWindows()
            print("[INFO] Cleanup completed")
        except Exception as cleanup_error:
            print(f"[WARNING] Error during cleanup: {cleanup_error}")
