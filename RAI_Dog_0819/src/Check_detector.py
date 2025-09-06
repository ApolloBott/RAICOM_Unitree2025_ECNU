#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import numpy as np
import time
from ArmControl import ArmControl
from Board import setBusServoPulse
Distance = 0
Distance1 = 0

def read_distance1(serial_port):
    global Distance1
    if serial_port.in_waiting >= 0:
        frame = serial_port.read(8)
        serial_port.reset_input_buffer()
        if frame[0] == 0x01 and frame[1] == 0x03 and frame[2] == 0x02:
            Distance1 = (frame[3] << 8) | frame[4]
            if Distance1 <= 30000:
                serial_port.reset_input_buffer()
                return Distance1
        else:
            serial_port.reset_input_buffer()
            return Distance1
    serial_port.reset_input_buffer()
    return Distance1
    
def read_distance(ser):
    global Distance
    if ser.in_waiting >= 0:
        frame = ser.read(16)
       # ser.reset_input_buffer()
        if frame[0] == 0x57:
            check_sum = 0
            for i in range(15):
                check_sum += frame[i]
                if (check_sum&0x00ff) == frame[15] and frame[8] != 0x00:
                    Distance = (frame[10] << 16) | (frame[9] <<8) | frame[8]
                    #print(f"Distance: {Distance} mm")
                    ser.reset_input_buffer()
                    return Distance
    ser.reset_input_buffer()
    return Distance

if __name__ == "__main__":
    print("Let me Check your Serial port & Laser Detectors")
    ser_arm   = serial.Serial('/dev/arm', 115200, timeout=1)   # 原 ttyUSB0 (1-2.1)
    #ser_arm   = serial.Serial('/dev/ttyUSB0',         115200, timeout=1)   # 原 ttyUSB0 (1-2.1)
    armcontrol = ArmControl()
    armcontrol.reset_arm(165, 850, 0, 150, 500, 0, ser_arm)
    time.sleep(0.5)
    setBusServoPulse(4, 500, 1000, ser_arm)
    time.sleep(0.5)
    setBusServoPulse(4, 150, 1000, ser_arm)
    time.sleep(1)
    # ser0 = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
    # ser1 = serial.Serial('/dev/ttyCH343USB1', 115200, timeout=1)
    # ser2 = serial.Serial('/dev/ttyCH343USB2', 115200, timeout=1)
    
    #ser_lidar = serial.Serial('/dev/lidar',       115200, timeout=1)   # 原 ttyUSB1 (1-2.3)
    ser_right = serial.Serial('/dev/right',       115200, timeout=1)   # 原 ttyUSB1 (1-2.3)
    ser_d0    = serial.Serial('/dev/detector0',   921600, timeout=1)   # serial=5959048178
    ser_d1    = serial.Serial('/dev/detector1',   115200, timeout=1)   # serial=5959047026
    #ser_d2    = serial.Serial('/dev/detector2',   115200, timeout=1)   # serial=5959050042
    #ser2 = serial.Serial('COM10', 115200, timeout=1)
    #ser3 = serial.Serial('COM11', 115200, timeout=1)
    arm_head_distance = []
    arm_middle_distance = []
    dog_right_distance = []
    for _ in range(3):  
        arm_head_distance.append(read_distance(ser_d0))
        arm_middle_distance.append(read_distance1(ser_d1))
     #   dog_head_distance.append(read_distance1(ser_d2))
        dog_right_distance.append(read_distance1(ser_right))
        time.sleep(0.5)
        
    if np.average(arm_head_distance) > np.average(arm_middle_distance) \
        and np.average(dog_right_distance) > 500:
        print("All the Detectors are ready :)")
        print(f"Arm: Head_distance = {np.average(arm_head_distance):.1f} mm")
        print(f"Arm: Middle_distance = {np.average(arm_middle_distance):.1f} mm")
      #  print(f"Dog: Head_distance = {np.average(dog_head_distance):.1f} mm")
        print(f"Dog: Right_distance = {np.average(dog_right_distance):.1f} mm")
    else:
        print("Fatal Error")
        print("Please check the Detectors!")
        print(f"Arm: Head_distance = {np.average(arm_head_distance):.1f} mm")
        print(f"Arm: Middle_distance = {np.average(arm_middle_distance):.1f} mm")
       # print(f"Dog: Head_distance = {np.average(dog_head_distance):.1f} mm")
        print(f"Dog: Right_distance = {np.average(dog_right_distance):.1f} mm")

    #armcontrol.arm_catch_and_put(ser_d0, ser_d1, ser_arm)

