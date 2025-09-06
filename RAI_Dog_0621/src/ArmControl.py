#!/usr/bin/env python3
# encoding: utf-8
import time
import serial
import numpy as np
from distance import read_distance
import cv2
from Board import setBusServoPulse

d1 = 0.10314916202
l1 = 0.12941763737
l2 = 0.12941763737
l3 = 0.05945583202 + 0.1120 - 0.02

class ArmControl:
    def __init__(self):
    
        self.fin_circle = False
        self.fin_crossing = False
        
    def scan_range(self,serial_port, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.1):
        best_pulse, best_dist = pulse_start, float('inf')
        for pulse in range(pulse_start, pulse_end + 1, pulse_step):
            setBusServoPulse(1, pulse, use_time)
            #time.sleep(delay)
            d = read_distance(serial_port) 
            if d is not None:
               
                if d < best_dist:
                    best_dist, best_pulse = d, pulse
        return best_pulse, best_dist

    def locate_target_angle(self,laser_port='/dev/ttyUSB0', baudrate=115200):
        ser = serial.Serial(laser_port, baudrate, timeout=1)
        time.sleep(0.1)
     
        cp, cd = self.scan_range(ser, 350, 550, 5)
       
        fs, fe = max(350, cp-5), min(550, cp+5)
        
        fp, fd = self.scan_range(ser, fs, fe, 1)
     
        if 200 <= fp <= 800:
            setBusServoPulse(1, fp, 500)
            ser.close()
            return fp
        else :
            ser.close()
            return None, None
    def locate_target_distance(self,laser_port='/dev/ttyUSB1', baudrate=115200):
        ser = serial.Serial(laser_port, baudrate, timeout=1)
        time.sleep(0.1)
        distances = []
     
        for i in range(20):
            d = read_distance(ser)  
            if d is not None:
                distances.append(d)
            else:
                pass
            time.sleep(0.05)
        ser.close()
        if len(distances) == 0:
            return None
        avg_dist = np.mean(distances)
       
        return avg_dist

    def control_arm(self,px, py, pz, nx, ny, th3_min=-120, th3_max=120, th3_step=1):
        pz = 0  
        solutions = []
        x1 = np.hypot(px, py)
        y1 = pz
        if px < 0.2:
            P2 = 500
        elif 0.2 <= px < 0.27:
            P2 = 400
        elif 0.27 <= px < 0.32:
            P2 = 350
        elif 0.32 <= px < 0.36:
            P2 = 300
        elif 0.36 <= px < 0.4:
            P2 = 200
        else:
            P2 = 100
        #P2 = -1123.5059 * (np.hypot(px,pz)) + 715.6031
        d1_deg = (P2 - 125) / (500 - 125) * 90
        fai = np.radians(d1_deg)
        print("d1_deg:", d1_deg)
        print("P2:", P2)
        A = x1 - l1 * np.cos(fai)
        B = y1 - l1 * np.sin(fai)
        th3 = -np.arccos((A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3))
        D = l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3)
        cosine = (A * (l2 + l3 * np.cos(th3)) + B * (l3 * np.sin(th3)))/D
        sine = (-A * (l3 * np.sin(th3)) + B * (l2 + l3 * np.cos(th3)))/D

        th2 = np.arctan2(sine, cosine) - fai

        d1_deg, d2_deg, d3_deg = np.degrees(fai), np.degrees(th2), np.degrees(th3)
        if 0 < d1_deg < 100 :
            P3 = 500/120 * (120 + d2_deg)
            P4 = 500 + (d3_deg/120)*500 if d3_deg>=0 else 500*(120 + d3_deg)/120
            print("f", {"P2":P2, "P3":P3, "P4":P4})
        else:
            return []
        req = [(2, P2), (3, P3), (4, P4)]
        for sid, pulse in req:
            pulse = int(round(pulse))
            setBusServoPulse(sid, pulse, 1000)
            time.sleep(0.2)
        time.sleep(1)
        setBusServoPulse(6, 800, 200)
      

    def reset_arm(self,P1, P2, P3, P4, P5=530, P6=100):
        reset_seq = [(1, P1), (2, P2), (3, P3), (4, P4)]
        for sid, pulse in reset_seq:
            setBusServoPulse(sid, pulse, 200)
            time.sleep(0.2)
        setBusServoPulse(6, P6, 200) 
        

    def arm_circle(self):
        reset_seq = [(1, 500), (2, 500), (3, 0), (4, 500), (5, 530), (6, 100)]
        for sid, pulse in reset_seq:
            setBusServoPulse(sid, pulse, 200)
       
        p1= self.locate_target_angle(laser_port='/dev/ttyUSB0', baudrate=115200)
        reset_seq = [ (3, 100), (4, 700)]
        for sid, pulse in reset_seq:
            setBusServoPulse(sid, pulse, 200)
        dist_mm = self.locate_target_distance(laser_port='/dev/ttyUSB1', baudrate=115200)
        offset = 50 
     
        px = (dist_mm + offset) / 1000.0
        py = 0.0
        pz = 0.0 
        self.control_arm(px, py, pz, nx=1.0, ny=0.0)
        
        time.sleep(2)
     
        reset_seq = [(1, 500), (2, 450), (3, 100), (4, 700)]
        for sid, pulse in reset_seq:
            setBusServoPulse(sid, pulse, 200)

        reset_seq = [(1, 125), (2, 450), (3, 0), (4, 700)]
        for sid, pulse in reset_seq:
            setBusServoPulse(sid, pulse, 800)
        time.sleep(1)
        setBusServoPulse(6, 100, 800) 
        self.fin_circle = True
        

if __name__ == "__main__":
    armcontrol = ArmControl()
    armcontrol.arm_circle()
    

