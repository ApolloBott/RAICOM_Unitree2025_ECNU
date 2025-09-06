from Img_Processor import Img_Processor
from Actions_crossing2 import Actions
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
        #position = data.position
        #print(f"position: {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}")
        rpy = data.imu_state.rpy
        pitch = rpy[1]
        #print(f"rpy: {rpy[0]:.3f}, {rpy[1]:.3f}, "
              #f"{rpy[2]:.3f}\n")
'''      
class RangeInfoHandler(ChannelSubscriber):
    def __init__(self):
        super().__init__(TOPIC_RANGE_INFO, PointStamped_)
    def on_data(self, data):
        global front,left,right
        front = data.point.x
        left = data.point.y
        right = data.point.z
        print(f"\trange front = {data.point.x:.3f}")
        print(f"\trange left = {data.point.y:.3f}")
        print(f"\trange right = {data.point.z:.3f}\n")
'''
        
if __name__ == "__main__":
    #global right
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)
    ChannelFactoryInitialize(0, sys.argv[1])
    sport_client = SportClient()  
    sport_client.SetTimeout(10.0)
    sport_client.Init()
    handler1 = HighStateHandler()
    
    # ser1 = serial.Serial('/dev/ttyCH343USB1', 921600, timeout=1)
    
    # ser2 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    
    #ser0.close()
    #ser1.close()
    #ser2.close()
    #ser2 = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
    #handler2 = RangeInfoHandler()
    img_processor = Img_Processor()
    camera_path = "/dev/video0"
    cap = cv2.VideoCapture(camera_path)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 440)   
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 440)  
    cap.set(cv2.CAP_PROP_FPS, 30) 
    actions = Actions()
    
    armcontrol = ArmControl()
    # ser0 = armcontrol.check_serial('/dev/ttyCH343USB0', 921600)
    # ser1 = armcontrol.check_serial('/dev/ttyCH343USB1', 921600)
    # ser2 = armcontrol.check_serial('/dev/ttyUSB0', 115200)
    # ser3 = armcontrol.check_serial('/dev/ttyUSB1', 115200)
    # ser4 = armcontrol.check_serial('/dev/ttyCH343USB2', 115200)
    ser0 = armcontrol.check_serial('/dev/detector0',   921600, timeout=1)   # serial=5959048178  机械臂 头顶测距
    ser1 = armcontrol.check_serial('/dev/detector1',   115200, timeout=1)   # serial=5959047026  机械臂 中部测距
    ser2 = armcontrol.check_serial('/dev/arm',         115200, timeout=1)   # 原 ttyUSB0 (1-2.1) 机械臂 总线舵机
    ser3 = armcontrol.check_serial('/dev/right',       115200, timeout=1)   # 原 ttyUSB1 (1-2.3) 右测距 避障使用
    #ser4 = armcontrol.check_serial('/dev/detector2',   115200, timeout=1)   # serial=5959050042  避障区 狗头测距
    if ser0 is None or ser1 is None or ser2 is None or ser3 is None or ser4 is None:
        print("[ERROR] Serial port connection failed. Please check the connections.")
        exit(1)  


    # ser3 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
    # ser4 = serial.Serial('/dev/ttyCH343USB2', 115200, timeout=1)
    ser3 = serial.Serial('/dev/right', 115200, timeout=1)
    #ser4 = serial.Serial('/dev/detector2', 115200, timeout=1)

    # 判断是否都连接成功

    armcontrol.reset(125, 850, 0, 150, 500, ser2)
    setBusServoPulse(4, 500, 1000, ser2)
    time.sleep(0.5)
    setBusServoPulse(4, 150, 1000, ser2)
    time.sleep(0.5)
    setBusServoPulse(4, 500, 1000, ser2)
    time.sleep(0.5)
    setBusServoPulse(4, 150, 1000, ser2)
    time.sleep(0.5)
    setBusServoPulse(6, 0, 1000, ser2)
    time.sleep(0.5)
    #armcontrol.reset_arm(125, 850, 0, 150, 500, 0, ser2)

    #img_processor.finish_visual = True
    actions.sm = "s_start"
    #timer.start_timer()
    #actions.micro_s = 9
    #img_processor.finish_visual = False
    #time.sleep(5)
    
    while True:  
        #print(f"right:{right} mm")
        if not img_processor.finish_visual:
            ret, frame = cap.read()
            if not ret:
                print("finish")
                break
            img_processor.image = frame
            img_processor.get_origin()
            img_processor.get_binary()
            img_processor.get_contours()
            img_processor.get_special_pnts()
            img_processor.get_evens()
            img_processor.get_width()
            #img_processor.recog_tag()
        print(f"s:{actions.micro_s}")
        print(f"sm:{actions.sm}")
        print(f"len:{len(img_processor.idss)}")
        print(f"idss:{img_processor.idss}")
        if actions.sm == 's_start':
            actions.f_start(img_processor,sport_client,armcontrol,ser3)
            #actions.f_start(img_processor,sport_client)
        elif actions.sm == 's_in_circle':
            actions.f_in_circle(img_processor,sport_client,armcontrol,ser0,ser1,ser2)
            #actions.f_in_circle(img_processor,sport_client)
        elif actions.sm == 's_in_crossing1':
            actions.f_in_crossing1(img_processor,sport_client,armcontrol,ser1,ser2)
            #actions.f_in_crossing1(img_processor,sport_client)
        elif actions.sm == 's_in_crossing2':
            actions.f_in_crossing2(img_processor,sport_client,armcontrol,ser1,ser2)
            #actions.f_in_crossing2(img_processor,sport_client)
        elif actions.sm == 's_ign_crossing':
            actions.f_ignore_crossing(img_processor,sport_client)
        elif actions.sm == 's_up':
            actions.f_up(img_processor,sport_client,pitch)
            #actions.f_up(sport_client,pitch)
        elif actions.sm == 's_finish':
            actions.f_finish(img_processor,sport_client)
        elif actions.sm == 's_straight':
            actions.f_go_straight(img_processor,sport_client)   
        if cv2.waitKey(1) == 27:
            ser0.close()
            ser1.close()
            ser2.close()
            print("closed")
            break
    cap.release()
    cv2.destroyAllWindows()

    
