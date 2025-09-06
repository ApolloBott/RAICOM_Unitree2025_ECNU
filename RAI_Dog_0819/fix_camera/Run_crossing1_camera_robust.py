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
from CameraManager import CameraManager

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

class RobustArmControl:
    """
    包装ArmControl的鲁棒性封装，处理机械臂操作和摄像头稳定性
    """
    def __init__(self, camera_manager):
        self.armcontrol = ArmControl()
        self.camera_manager = camera_manager
        self.last_arm_operation_time = 0
        self.camera_stabilization_delay = 1.0  # 机械臂操作后摄像头稳定延迟
    
    def check_serial(self, *args, **kwargs):
        """代理串口检查方法"""
        return self.armcontrol.check_serial(*args, **kwargs)
    
    def reset(self, *args, **kwargs):
        """代理reset方法，操作后检查摄像头"""
        result = self.armcontrol.reset(*args, **kwargs)
        self._post_arm_operation()
        return result
    
    def reset_arm(self, *args, **kwargs):
        """代理reset_arm方法，操作后检查摄像头"""
        result = self.armcontrol.reset_arm(*args, **kwargs)
        self._post_arm_operation()
        return result
    
    def arm_put_fixed(self, *args, **kwargs):
        """代理arm_put_fixed方法，操作后检查摄像头"""
        result = self.armcontrol.arm_put_fixed(*args, **kwargs)
        self._post_arm_operation()
        return result
    
    def arm_catch_and_put(self, *args, **kwargs):
        """代理arm_catch_and_put方法，操作后检查摄像头"""
        result = self.armcontrol.arm_catch_and_put(*args, **kwargs)
        self._post_arm_operation()
        return result
    
    def _post_arm_operation(self):
        """机械臂操作后的处理"""
        self.last_arm_operation_time = time.time()
        
        # 先检查摄像头当前状态
        if not self.camera_manager.is_camera_stable():
            print("[WARNING] Camera appears unstable after arm operation, attempting reconnection...")
            
            # 尝试快速重连，如果失败再等待
            success = self.camera_manager.connect()
            if not success:
                print("[INFO] Quick reconnection failed, waiting for device stabilization...")
                time.sleep(self.camera_stabilization_delay)
                self.camera_manager.connect()
        else:
            # 摄像头看起来正常，只做轻量级检查
            try:
                ret, frame = self.camera_manager.read_frame()
                if not ret or frame is None:
                    print("[WARNING] Camera read test failed, attempting reconnection...")
                    self.camera_manager.connect()
            except Exception as e:
                print(f"[WARNING] Camera test failed: {e}, attempting reconnection...")
                self.camera_manager.connect()
    
    def __getattr__(self, name):
        """代理其他所有方法到原始ArmControl"""
        return getattr(self.armcontrol, name)

if __name__ == "__main__":
    #global right
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)
    
    try:
        ChannelFactoryInitialize(0, sys.argv[1])
        sport_client = SportClient()  
        sport_client.SetTimeout(10.0)
        sport_client.Init()
        handler1 = HighStateHandler()
        
        # 初始化摄像头管理器
        print("[INFO] Initializing camera manager...")
        camera_manager = CameraManager(
            preferred_path="/dev/video0",
            backup_paths=["/dev/camera0", "/dev/cameranew", "/dev/camera_port4", "/dev/video1"],
            width=440,
            height=440,
            fps=30
        )
        
        if not camera_manager.is_camera_stable():
            print("[ERROR] Failed to initialize camera")
            sys.exit(1)
        
        print(f"[INFO] {camera_manager.get_camera_info()}")
        
        img_processor = Img_Processor()
        actions = Actions()
        
        # 使用鲁棒性包装的机械臂控制
        armcontrol = RobustArmControl(camera_manager)
        
        # 串口连接检查
        print("[INFO] Checking serial connections...")
        ser0 = armcontrol.check_serial('/dev/detector0',   921600, timeout=1)   # serial=5959048178  机械臂 头顶测距
        ser1 = armcontrol.check_serial('/dev/detector1',   115200, timeout=1)   # serial=5959047026  机械臂 中部测距
        ser2 = armcontrol.check_serial('/dev/arm',         115200, timeout=1)   # 原 ttyUSB0 (1-2.1) 机械臂 总线舵机
        ser3 = armcontrol.check_serial('/dev/right',       115200, timeout=1)   # 原 ttyUSB1 (1-2.3) 右测距 避障使用
      
        if ser3 is None or ser2 is None or ser0 is None or ser1 is None:
            print("[ERROR] Serial port connection failed. Please check the connections.")
            sys.exit(1)

        # 重新打开必要的串口连接
        ser3 = serial.Serial('/dev/right', 115200, timeout=1)   

        # 初始化机械臂
        print("[INFO] Initializing robotic arm...")
        armcontrol.reset(125, 850, 0, 150, 500, ser2)
        setBusServoPulse(4, 300, 1000, ser2)
        time.sleep(0.2)
        armcontrol.reset_arm(125, 850, 0, 150, 500, 0, ser2)
        
        # 状态机初始化
        actions.sm = "s_start"
        
        print("[INFO] Starting main control loop...")
        frame_count = 0
        last_camera_check = time.time()
        
        while True:  
            # 定期检查摄像头状态
            current_time = time.time()
            if current_time - last_camera_check > 5.0:  # 每5秒检查一次
                if not camera_manager.is_camera_stable():
                    print("[WARNING] Camera stability check failed, attempting reconnection...")
                    camera_manager.connect()
                last_camera_check = current_time
            
            # 图像处理
            if not img_processor.finish_visual:
                ret, frame = camera_manager.read_frame()
                if not ret or frame is None:
                    print("[WARNING] Camera read failed, skipping frame...")
                    time.sleep(0.1)  # 短暂等待后继续
                    continue
                
                frame_count += 1
                if frame_count % 30 == 0:  # 每30帧打印一次状态
                    print(f"[INFO] Processed {frame_count} frames, camera: {camera_manager.current_path}")
                
                img_processor.image = frame
                img_processor.get_origin()
                img_processor.get_binary()
                img_processor.get_contours()
                img_processor.get_special_pnts()
                img_processor.get_evens()
                img_processor.get_width()
                #img_processor.recog_tag()
            
            # 状态机处理
            print(f"s:{actions.micro_s}")
            print(f"sm:{actions.sm}")
            print(f"len:{len(img_processor.idss)}")
            print(f"idss:{img_processor.idss}")
            
            if actions.sm == 's_start':
                actions.f_start(img_processor,sport_client,armcontrol,ser3)
            elif actions.sm == 's_in_circle':
                actions.f_in_circle(img_processor,sport_client,armcontrol,ser0,ser1,ser2)
            elif actions.sm == 's_in_crossing1':
                actions.f_in_crossing1(img_processor,sport_client,armcontrol,ser1,ser2)
            elif actions.sm == 's_in_crossing2':
                actions.f_in_crossing2(img_processor,sport_client,armcontrol,ser1,ser2)
            elif actions.sm == 's_ign_crossing':
                actions.f_ignore_crossing(img_processor,sport_client)
            elif actions.sm == 's_up':
                actions.f_up(img_processor,sport_client,pitch)
            elif actions.sm == 's_finish':
                actions.f_finish(img_processor,sport_client)
            elif actions.sm == 's_straight':
                actions.f_go_straight(img_processor,sport_client)   
            
            if cv2.waitKey(1) == 27:
                break
                
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
    finally:
        # 清理资源
        print("[INFO] Cleaning up resources...")
        try:
            if 'ser0' in locals() and ser0: ser0.close()
            if 'ser1' in locals() and ser1: ser1.close()  
            if 'ser2' in locals() and ser2: ser2.close()
            if 'ser3' in locals() and ser3: ser3.close()
         
            if 'camera_manager' in locals(): camera_manager.close()
            print("[INFO] All resources cleaned up")
        except Exception as e:
            print(f"[WARNING] Error during cleanup: {e}")
        
        cv2.destroyAllWindows()
