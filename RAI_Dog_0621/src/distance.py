#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time
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
                    print(f"Distance: {Distance} mm")
                    ser.reset_input_buffer()
                    return Distance
    ser.reset_input_buffer()
    return Distance

if __name__ == "__main__":

    ser1 = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
    #ser2 = serial.Serial('COM10', 115200, timeout=1)
    #ser3 = serial.Serial('COM11', 115200, timeout=1)
    while True:
        left_distance = read_distance(ser1)
        print(left_distance)
        #time.sleep(0.4)
    #right_distance = read_distance(ser2)
    #front_distance = read_distance(ser3)
       # print(left_distance)