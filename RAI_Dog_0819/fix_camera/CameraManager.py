import cv2
import time
import os
import subprocess
import glob
import threading
from typing import Optional, Tuple

class CameraManager:
    """
    摄像头管理器，用于处理摄像头断线重连和稳定性管理
    专门针对USB摄像头在机械臂操作后可能掉线的问题
    """
    
    def __init__(self, preferred_path="/dev/video0", backup_paths=None, width=440, height=440, fps=30):
        self.preferred_path = preferred_path
        # 根据新的udev规则更新备用路径
        self.backup_paths = backup_paths or [
            "/dev/camera0",      # udev规则创建的主摄像头链接
            "/dev/cameranew",    # 基于序列号的摄像头链接
            "/dev/camera_port4", # USB端口4的摄像头
            "/dev/camera_port5", # USB端口5的摄像头
            "/dev/video1", 
            "/dev/video2"
        ]
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.current_path = None
        self.last_successful_read_time = time.time()
        self.reconnection_delay = 2.0
        self.max_failed_reads = 5
        self.failed_read_count = 0
        self.is_stable = False
        
        # 初始化摄像头
        self.connect()
    
    def find_available_cameras(self):
        """查找所有可用的摄像头设备"""
        available_cameras = []
        
        # 检查常见的摄像头路径
        paths_to_check = [self.preferred_path] + self.backup_paths
        
        for path in paths_to_check:
            if os.path.exists(path):
                try:
                    test_cap = cv2.VideoCapture(path)
                    if test_cap.isOpened():
                        ret, _ = test_cap.read()
                        if ret:
                            available_cameras.append(path)
                            print(f"[INFO] Found working camera: {path}")
                    test_cap.release()
                except Exception as e:
                    print(f"[DEBUG] Camera test failed for {path}: {e}")
        
        # 如果没找到，尝试通过数字索引查找
        if not available_cameras:
            for i in range(4):
                try:
                    test_cap = cv2.VideoCapture(i)
                    if test_cap.isOpened():
                        ret, _ = test_cap.read()
                        if ret:
                            available_cameras.append(i)
                            print(f"[INFO] Found working camera at index: {i}")
                    test_cap.release()
                except Exception as e:
                    continue
        
        return available_cameras
    
    def reset_usb_camera_power(self, device_path):
        """尝试重置USB摄像头电源状态"""
        try:
            # 查找USB设备信息
            cmd = f"udevadm info --name={device_path} --query=all | grep ID_PATH"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            if result.returncode == 0 and result.stdout:
                # 提取USB路径信息
                usb_path = result.stdout.split('=')[-1].strip()
                print(f"[INFO] Trying to reset USB power for device: {usb_path}")
                
                # 尝试通过sysfs重置USB端口
                usb_reset_cmd = f"echo 0 > /sys/bus/usb/devices/{usb_path}/authorized; sleep 1; echo 1 > /sys/bus/usb/devices/{usb_path}/authorized"
                subprocess.run(usb_reset_cmd, shell=True)
                
        except Exception as e:
            print(f"[DEBUG] USB reset failed: {e}")
    
    def connect(self):
        """连接到摄像头，优先使用首选路径，失败则尝试备用路径"""
        # 释放当前连接
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            time.sleep(0.5)  # 等待设备释放
        
        # 查找可用摄像头
        available_cameras = self.find_available_cameras()
        
        if not available_cameras:
            print("[ERROR] No working cameras found!")
            return False
        
        # 按优先级尝试连接
        for camera_path in available_cameras:
            try:
                print(f"[INFO] Attempting to connect to camera: {camera_path}")
                
                # 如果是字符串路径，先检查USB电源状态
                if isinstance(camera_path, str) and "/dev/" in camera_path:
                    self.reset_usb_camera_power(camera_path)
                    time.sleep(1.0)  # 等待设备稳定
                
                self.cap = cv2.VideoCapture(camera_path)
                
                if not self.cap.isOpened():
                    print(f"[WARNING] Could not open camera: {camera_path}")
                    continue
                
                # 设置摄像头参数
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                # 设置缓冲区大小避免延迟
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # 测试读取
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.current_path = camera_path
                    self.failed_read_count = 0
                    self.last_successful_read_time = time.time()
                    self.is_stable = True
                    print(f"[SUCCESS] Camera connected successfully: {camera_path}")
                    print(f"[INFO] Camera resolution: {frame.shape[1]}x{frame.shape[0]}")
                    return True
                else:
                    print(f"[WARNING] Camera opened but cannot read: {camera_path}")
                    self.cap.release()
                    
            except Exception as e:
                print(f"[ERROR] Failed to connect to camera {camera_path}: {e}")
                if self.cap:
                    self.cap.release()
                    self.cap = None
        
        print("[ERROR] Failed to connect to any camera")
        self.is_stable = False
        return False
    
    def read_frame(self) -> Tuple[bool, Optional[object]]:
        """
        读取摄像头帧，带有自动重连功能
        返回: (成功标志, 图像帧)
        """
        if self.cap is None or not self.cap.isOpened():
            print("[WARNING] Camera not connected, attempting to reconnect...")
            if not self.connect():
                return False, None
        
        try:
            ret, frame = self.cap.read()
            
            if ret and frame is not None:
                self.failed_read_count = 0
                self.last_successful_read_time = time.time()
                self.is_stable = True
                return True, frame
            else:
                self.failed_read_count += 1
                print(f"[WARNING] Camera read failed (attempt {self.failed_read_count}/{self.max_failed_reads})")
                
                # 如果连续失败次数过多，尝试重连
                if self.failed_read_count >= self.max_failed_reads:
                    print("[WARNING] Too many failed reads, attempting to reconnect camera...")
                    self.is_stable = False
                    time.sleep(self.reconnection_delay)
                    if self.connect():
                        return self.read_frame()
                    else:
                        return False, None
                
                return False, None
                
        except Exception as e:
            print(f"[ERROR] Camera read exception: {e}")
            self.failed_read_count += 1
            self.is_stable = False
            
            if self.failed_read_count >= self.max_failed_reads:
                print("[ERROR] Camera completely failed, attempting full reconnection...")
                time.sleep(self.reconnection_delay)
                if not self.connect():
                    return False, None
            
            return False, None
    
    def is_camera_stable(self) -> bool:
        """检查摄像头是否稳定"""
        if not self.is_stable:
            return False
        
        # 检查最后成功读取的时间
        time_since_last_read = time.time() - self.last_successful_read_time
        if time_since_last_read > 10.0:  # 超过10秒没有成功读取
            return False
        
        return self.cap is not None and self.cap.isOpened()
    
    def close(self):
        """关闭摄像头"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            print("[INFO] Camera closed")
    
    def refresh_status(self):
        """刷新摄像头状态，用于长时间操作后避免误判"""
        self.last_successful_read_time = time.time() - 1  # 设置为1秒前，避免超时误判
        self.failed_read_count = 0
        self.is_stable = True
        print("[INFO] Camera status refreshed")
    
    def get_camera_info(self):
        """获取当前摄像头信息"""
        if self.cap is None:
            return "No camera connected"
        
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        return f"Camera: {self.current_path}, Resolution: {width}x{height}, FPS: {fps}"

# 测试函数
if __name__ == "__main__":
    camera_manager = CameraManager()
    
    print("Testing camera manager...")
    print(camera_manager.get_camera_info())
    
    # 测试读取几帧
    for i in range(10):
        ret, frame = camera_manager.read_frame()
        if ret:
            print(f"Frame {i+1}: {frame.shape}")
        else:
            print(f"Frame {i+1}: Failed")
        time.sleep(0.1)
    
    camera_manager.close()
