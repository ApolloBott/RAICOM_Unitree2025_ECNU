#!/usr/bin/env python3

import os
import subprocess
import time
import glob
import cv2

class USBDeviceReset:
    """USB设备重置工具，专门用于重置摄像头设备"""
    
    @staticmethod
    def find_usb_device_by_video_device(video_device):
        """根据video设备找到对应的USB设备目录"""
        try:
            # 使用udevadm获取设备信息
            cmd = f"udevadm info --name={video_device} --query=all"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            if result.returncode != 0:
                return None
            
            vendor_id = None
            product_id = None
            
            for line in result.stdout.split('\n'):
                if 'ID_VENDOR_ID' in line:
                    vendor_id = line.split('=')[-1].strip()
                elif 'ID_MODEL_ID' in line:
                    product_id = line.split('=')[-1].strip()
            
            if not vendor_id or not product_id:
                return None
            
            # 查找对应的USB设备目录
            usb_devices = glob.glob('/sys/bus/usb/devices/*/')
            for usb_dir in usb_devices:
                try:
                    vendor_file = os.path.join(usb_dir, 'idVendor')
                    product_file = os.path.join(usb_dir, 'idProduct')
                    
                    if os.path.exists(vendor_file) and os.path.exists(product_file):
                        with open(vendor_file, 'r') as f:
                            dev_vendor = f.read().strip()
                        with open(product_file, 'r') as f:
                            dev_product = f.read().strip()
                        
                        if dev_vendor == vendor_id and dev_product == product_id:
                            return usb_dir, vendor_id, product_id
                except:
                    continue
                    
            return None
            
        except Exception as e:
            print(f"Error finding USB device: {e}")
            return None
    
    @staticmethod
    def reset_usb_device_power(usb_dir):
        """重置USB设备电源"""
        try:
            authorized_file = os.path.join(usb_dir, 'authorized')
            if not os.path.exists(authorized_file):
                return False
            
            print(f"Resetting USB device: {usb_dir}")
            
            # 尝试重置设备
            try:
                # 禁用设备
                subprocess.run(f"echo 0 | sudo tee {authorized_file}", shell=True, check=True)
                time.sleep(1)
                # 重新启用设备
                subprocess.run(f"echo 1 | sudo tee {authorized_file}", shell=True, check=True)
                print("USB device power reset successful")
                return True
            except subprocess.CalledProcessError:
                print("Failed to reset device power (permission denied)")
                return False
                
        except Exception as e:
            print(f"Error resetting USB device: {e}")
            return False
    
    @staticmethod
    def reset_uvc_driver():
        """重新加载UVC驱动"""
        try:
            print("Reloading UVC driver...")
            # 卸载驱动
            subprocess.run("sudo modprobe -r uvcvideo", shell=True)
            time.sleep(2)
            # 重新加载驱动
            subprocess.run("sudo modprobe uvcvideo", shell=True)
            time.sleep(1)
            print("UVC driver reloaded")
            return True
        except Exception as e:
            print(f"Failed to reload UVC driver: {e}")
            return False
    
    @staticmethod
    def reset_camera_device(video_device="/dev/video0"):
        """重置摄像头设备的完整流程"""
        print(f"=== Resetting camera device: {video_device} ===")
        
        # 检查设备是否存在
        if not os.path.exists(video_device):
            print(f"Device {video_device} does not exist")
            return False
        
        # 测试设备是否可用
        print("Testing camera before reset...")
        try:
            cap = cv2.VideoCapture(video_device)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None:
                    print("Camera is working normally, no reset needed")
                    return True
                else:
                    print("Camera exists but cannot read frames")
            else:
                print("Camera cannot be opened")
        except Exception as e:
            print(f"Camera test failed: {e}")
        
        # 查找USB设备
        usb_info = USBDeviceReset.find_usb_device_by_video_device(video_device)
        if usb_info:
            usb_dir, vendor_id, product_id = usb_info
            print(f"Found USB device: {vendor_id}:{product_id} at {usb_dir}")
            
            # 尝试重置USB设备电源
            if USBDeviceReset.reset_usb_device_power(usb_dir):
                time.sleep(3)  # 等待设备重新初始化
                
                # 测试重置后的效果
                try:
                    cap = cv2.VideoCapture(video_device)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        cap.release()
                        if ret and frame is not None:
                            print("Camera reset successful!")
                            return True
                except:
                    pass
        
        # USB设备重置失败，尝试驱动重载
        print("USB device reset failed, trying driver reload...")
        if USBDeviceReset.reset_uvc_driver():
            time.sleep(3)
            
            # 测试驱动重载后的效果
            try:
                cap = cv2.VideoCapture(video_device)
                if cap.isOpened():
                    ret, frame = cap.read()
                    cap.release()
                    if ret and frame is not None:
                        print("Camera reset successful after driver reload!")
                        return True
            except:
                pass
        
        print("All reset methods failed")
        return False

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='USB Camera Device Reset Tool')
    parser.add_argument('--device', '-d', default='/dev/video0',
                       help='Video device to reset (default: /dev/video0)')
    parser.add_argument('--test', '-t', action='store_true',
                       help='Test camera without resetting')
    
    args = parser.parse_args()
    
    if args.test:
        print(f"Testing camera device: {args.device}")
        try:
            cap = cv2.VideoCapture(args.device)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None:
                    print(f"✓ Camera {args.device} is working - Resolution: {frame.shape[1]}x{frame.shape[0]}")
                else:
                    print(f"✗ Camera {args.device} opened but cannot read frames")
            else:
                print(f"✗ Camera {args.device} cannot be opened")
        except Exception as e:
            print(f"✗ Camera test failed: {e}")
    else:
        success = USBDeviceReset.reset_camera_device(args.device)
        if success:
            print("Camera reset completed successfully")
        else:
            print("Camera reset failed")

if __name__ == "__main__":
    main()
