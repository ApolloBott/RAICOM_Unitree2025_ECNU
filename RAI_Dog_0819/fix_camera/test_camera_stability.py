#!/usr/bin/env python3

"""
快速测试摄像头稳定性和重连功能
"""

import sys
import time
import cv2
from CameraManager import CameraManager

def test_camera_stability():
    """测试摄像头稳定性"""
    print("=== Camera Stability Test ===")
    
    # 初始化摄像头管理器
    camera_manager = CameraManager(
        preferred_path="/dev/video0",
        backup_paths=["/dev/camera0", "/dev/cameranew", "/dev/camera_port4", "/dev/video1"],
        width=440,
        height=440,
        fps=30
    )
    
    if not camera_manager.is_camera_stable():
        print("❌ Failed to initialize camera")
        return False
    
    print(f"✅ Camera initialized: {camera_manager.get_camera_info()}")
    
    # 测试连续读取
    print("\n--- Testing continuous frame reading ---")
    for i in range(20):
        ret, frame = camera_manager.read_frame()
        if ret and frame is not None:
            print(f"Frame {i+1:2d}: ✅ {frame.shape}")
        else:
            print(f"Frame {i+1:2d}: ❌ Failed")
            
            # 模拟重连测试
            print("Attempting reconnection...")
            if camera_manager.connect():
                print("✅ Reconnection successful")
            else:
                print("❌ Reconnection failed")
                break
        
        time.sleep(0.1)
    
    # 测试USB重置功能
    print("\n--- Testing USB reset functionality ---")
    try:
        camera_manager.reset_usb_camera_power(camera_manager.current_path)
        time.sleep(2)
        
        # 测试重置后的功能
        ret, frame = camera_manager.read_frame()
        if ret and frame is not None:
            print("✅ Camera working after USB reset")
        else:
            print("❌ Camera failed after USB reset")
            camera_manager.connect()
            
    except Exception as e:
        print(f"❌ USB reset test failed: {e}")
    
    # 清理
    camera_manager.close()
    print("\n✅ Test completed")
    return True

def simulate_arm_operation():
    """模拟机械臂操作对摄像头的影响"""
    print("\n=== Simulating Arm Operation Impact ===")
    
    camera_manager = CameraManager()
    
    if not camera_manager.is_camera_stable():
        print("❌ Failed to initialize camera for simulation")
        return False
    
    print("📹 Camera working normally")
    
    # 读取几帧正常帧
    for i in range(5):
        ret, frame = camera_manager.read_frame()
        if ret:
            print(f"Pre-operation frame {i+1}: ✅")
        else:
            print(f"Pre-operation frame {i+1}: ❌")
        time.sleep(0.1)
    
    print("\n🤖 Simulating arm operation (USB stress)...")
    
    # 模拟机械臂操作后的USB重置
    camera_manager.reset_usb_camera_power(camera_manager.current_path)
    
    # 检查摄像头稳定性
    if not camera_manager.is_camera_stable():
        print("⚠️  Camera appears unstable after operation")
        if camera_manager.connect():
            print("✅ Auto-reconnection successful")
        else:
            print("❌ Auto-reconnection failed")
    
    # 读取操作后的帧
    print("\n📹 Testing post-operation camera...")
    for i in range(5):
        ret, frame = camera_manager.read_frame()
        if ret:
            print(f"Post-operation frame {i+1}: ✅")
        else:
            print(f"Post-operation frame {i+1}: ❌")
            # 尝试重连
            camera_manager.connect()
        time.sleep(0.1)
    
    camera_manager.close()
    return True

def main():
    print("🔧 RAI Dog Camera Stability Test Tool")
    print("=====================================")
    
    if len(sys.argv) > 1 and sys.argv[1] == "--arm-sim":
        simulate_arm_operation()
    else:
        test_camera_stability()
    
    print("\n💡 Usage:")
    print("  python3 test_camera_stability.py           # Basic stability test")
    print("  python3 test_camera_stability.py --arm-sim # Simulate arm operation impact")

if __name__ == "__main__":
    main()
