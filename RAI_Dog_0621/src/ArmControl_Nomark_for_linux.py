#change all the serial port to ser0 and ser1

import time
import serial
import numpy as np
from distance import read_distance
from distance import read_distance1
import cv2
from BusServoCmd import *

#DH Parameters of the robotic arm
d1 = 0.10314916202
l1 = 0.12941763737
l2 = 0.12941763737
l3 = 0.05945583202 + 0.1120 
ser0 = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
ser1 = serial.Serial('/dev/ttyCH343USB1', 921600, timeout=1)
ser2 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def setBusServoPulse(id, pulse, use_time, ser2):
    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(ser2,id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)
    return pulse
    
class ArmControl:
    def __init__(self):
        self.fin_circle = False
        self.fin_crossing = False
    '''
    def scan_range(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.2):
        
        best_pulse, best_dist = pulse_start, float('inf')
        for pulse in range(pulse_start, pulse_end + 1, pulse_step):
            setBusServoPulse(1, pulse, use_time, ser2)
            time.sleep(delay)
            #Change to New Laser Detect module
            d = read_distance(serial_port)  
            print(d)
            if d is not None and d < 200:
                if d < best_dist and d > 50:
                    best_dist, best_pulse = d, pulse
        return best_pulse, best_dist
    '''
    
    def scan_range(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.2):
        
        best_pulse, best_dist = pulse_start, float('inf')
        for pulse in range(pulse_start, pulse_end + 1, pulse_step):
            setBusServoPulse(1, pulse, use_time, ser2)
            readings = []
            for _ in range(5):
                d = read_distance(serial_port)  
                print(f"raw:{d}")
                if d is not None:
                    readings.append(d)
                #time.sleep(0.05)
            if readings:
                filtered_d = np.mean(readings)
                print(filtered_d)
                print(np.var(readings))
                if filtered_d < best_dist and filtered_d >50 and np.var(readings) < 10:
                    best_dist, best_pulse = filtered_d, pulse
        return best_pulse, best_dist
    '''
    def scan_range(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.2):
        best_pulse, best_dist = pulse_start, float('inf')
        for pulse in range(pulse_start, pulse_end + 1, pulse_step):
            setBusServoPulse(1, pulse, use_time, ser2)
             
            window = []           
            window_size = 5      
            max_readings = 15   
            filtered_d = None
            stable = False   
            
            for _ in range(max_readings):
                d = read_distance(serial_port)
                if d is None: 
                    continue
                    
               
                window.append(d)
                if len(window) > window_size:
                    window.pop(0) 
                    
         
                if len(window) >= window_size:
                    filtered_d = np.mean(window)
                    window_variance = np.var(window)
                    print(f"Pulse {pulse}: raw={d:.1f}, filtered={filtered_d:.1f}, var={window_variance:.1f}")
                    
                   
                else:
                    print(f"Pulse {pulse}: raw={d:.1f} (filling window)")
            
       
            if filtered_d is None and window:
                filtered_d = np.mean(window)
                window_variance = np.var(window)
                print(f"Pulse {pulse}: final filtered={filtered_d:.1f}, var={window_variance:.1f}")
            
       
            if filtered_d is not None:
                and filtered_d < best_dist
                and filtered_d > 50
                and (stable or (window and np.var(window) < 20))):
                best_dist, best_pulse = filtered_d, pulse
        
        return best_pulse, best_dist   
       
    '''

    def scan_range_short(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.2):
        
        best_pulse, best_dist = pulse_start, float('inf')
        for pulse in range(pulse_start, pulse_end + 1, pulse_step):
            setBusServoPulse(1, pulse, use_time, ser2)
            time.sleep(delay)
            #Change to New Laser Detect module
            d = read_distance(serial_port)  
            print(d)
            if d is not None and d < 200:
                if d < best_dist and d > 50:
                    best_dist, best_pulse = d, pulse
        return best_pulse, best_dist
    '''
    def scan_range_for_circle(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.15):
        best_pulse, best_dist = pulse_start, float('inf')
        i = 0
        for pulse in range(pulse_start, pulse_end + 1, pulse_step):
            i+=1
            setBusServoPulse(1, pulse, use_time, ser2)
            readings = []
            for _ in range(7):
                if i > 5:
                    d = read_distance(serial_port)  
                else:
                    d = 0
                if d > 300 and d < 500:
                    readings.append(d)
                time.sleep(0.02)
            if readings:
                filtered_d = np.mean(readings)
                print(filtered_d)
                if len(readings) == 15:
                    best_dist, best_pulse = filtered_d, pulse
                    break
                    
        return best_dist, best_pulse
    '''
    def scan_range_for_circle(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.15):
        best_pulse, best_dist = pulse_start, float('inf')
        
        for pulse in range(pulse_start, pulse_end + 1, pulse_step):
            setBusServoPulse(1, pulse, use_time, ser2)
            readings = []
            
            
            read_count = 0
            while len(readings) < 15:
                d = read_distance(serial_port)
                read_count += 1
                
                
                if read_count <= 5:
                    continue
                    
            
                if 300 < d < 500:
                    readings.append(d)
                
                time.sleep(0.02)
            
         
            filtered_d = np.mean(readings)
            print(f"Pulse: {pulse}mm, Distance: {filtered_d:.1f}mm")
            
      
            best_dist, best_pulse = filtered_d, pulse
            break  
        
        return best_dist, best_pulse
    #def locate_target_angle(self, ser0, ser2, baudrate=115200):
    def locate_target_angle(self, ser0, ser2, baudrate=921600):
        """
        Locate the target angle using a laser distance sensor.
        Output:
        Returns the best pulse width for servo 1.
        """
        #ser = serial.Serial(laser_port, baudrate, timeout=1)
        #time.sleep(0.02)
       
        cp, cd = self.scan_range(ser0, ser2, 350, 650, 5)
        #time.sleep(0.1)
        fs, fe = max(300, cp-5), min(700, cp+5)
        
        fp, fd = self.scan_range(ser0, ser2, fs, fe, 1)
        time.sleep(0.1)
        if 200 <= fp <= 800:
            setBusServoPulse(1, fp, 1000, ser2)
            time.sleep(0.15)
            #ser.close()
            return fp  
    #def locate_target_angle_range_larger(self, ser0, ser2, baudrate=115200):
    def locate_target_angle_range_larger(self, ser0, ser2, baudrate=921600):
        #ser = serial.Serial(laser_port, baudrate, timeout=1)
        time.sleep(0.1)
       
        cp, cd = self.scan_range(ser0, ser2, 300, 650, 5)
        
        fs, fe = max(300, cp-5), min(650, cp+5)
        
        fp, fd = self.scan_range_short(ser0, ser2, fs, fe, 1)
      
        if 200 <= fp <= 800:
            setBusServoPulse(1, fp, 1000, ser2)
            #ser.close()
            return fp  
    #def locate_target_angle_for_circle(self, ser0, ser2, baudrate=115200):
    def locate_target_angle_for_circle(self, ser0, ser2, baudrate=921600):
    
        #ser = serial.Serial(laser_port, baudrate, timeout=1)
        time.sleep(0.1)
       
        fp, fd = self.scan_range_for_circle(ser1, ser2, 300, 650, 5)
        if 300 <= fp <= 650:
            setBusServoPulse(1, fp, 1000, ser2)
            #ser.close()
            return fp  
    '''
    #def locate_target_distance(self, ser1, ser2, baudrate=115200):
    def locate_target_distance(self, ser1, ser2, baudrate=921600):
        #ser = serial.Serial(laser_port, baudrate, timeout=1)
        time.sleep(0.1)
        distances = []
    
        for i in range(20):
            #d = read_distance(ser1) 
            #Change to New Laser Detect module 
            time.sleep(0.05)
            ser1 = serial.Serial('/dev/ttyCH343USB1', 115200, timeout=1)
            d = read_distance1(ser1) 
            if d is not None and d < 900 and d > 100:
                print(f"distance={d}")
                distances.append(d)
            
            time.sleep(0.05)
        #ser.close()
        if len(distances) == 0:
            
            return None
        avg_dist = np.mean(distances)
        
        return avg_dist
    '''

    def locate_target_distance(self, ser1, ser2, baudrate=921600):
        
        time.sleep(0.1)
        distances = []

        for _ in range(20):
            try:
                time.sleep(0.05)
               
                ser1 = serial.Serial('/dev/ttyCH343USB1', 115200, timeout=1)

                d = read_distance1(ser1)         
                if d is not None and 100 < d < 900:
                    print(f"distance = {d}")
                    distances.append(d)

            except (IndexError, serial.SerialException) as e:
                print(f"Fail{e},Use 270")
                distances.append(250)

            finally:
                if ser1 and ser1.is_open:
                    ser1.close()
            time.sleep(0.05)

        if len(distances) == 0:
            return 250

        avg_dist = np.mean(distances)
        return avg_dist

    def batch_solve_theta(self, px, py, pz, th3_min=-120, th3_max=120, th3_step=1):
        solutions = []
        x1 = (px**2 + py**2)**0.5  
        y1 = pz                
        for th3_deg in range(th3_min, th3_max+1, th3_step):
            th3 = np.deg2rad(th3_deg)
            A = 2 * l1 * (l2 + l3 * np.cos(th3))
            B = -2 * l1 * l3 * np.sin(th3)
            C = x1**2 + y1**2 - (l1**2 + l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3))
            R = np.hypot(A, B)
            if abs(C / R) <= 1:
                phi   = np.arctan2(B, A)
                delta = np.arccos(C / R)
                for sign in (+1, -1):
                    th2 = phi + sign * delta
                    A_ = l1 + l2 * np.cos(th2) + l3 * np.cos(th2 + th3)
                    B_ =       l2 * np.sin(th2) + l3 * np.sin(th2 + th3)
                    th1 = np.arctan2(A_ * y1 - B_ * x1, A_ * x1 + B_ * y1)
                    solutions.append((th1, th2, th3))
        return solutions
    
    def control_arm(self, px, py, pz, ser2, nx = 1.0, ny = 0, th3_min=-120, th3_max=120, th3_step=1):
        
        pz = -0.05 
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
        else:
            P2 = 200
        #P2 = -1123.5059 * (np.hypot(px,pz)) + 715.6031
        d1_deg = (P2 - 125) / (500 - 125) * 90
        fai = np.radians(d1_deg)
        
        A = x1 - l1 * np.cos(fai)
        B = y1 - l1 * np.sin(fai)
        #th3 = -np.arccos((A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3))
        cos_val = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)

        if not np.isfinite(cos_val) or cos_val < -1.0 or cos_val > 1.0:
            print("Use previous method")
            sols = self.batch_solve_theta(px, py, pz)
            best, best_sum = None, float('inf')
            for th1, th2, th3 in sols:
                d1_deg, d2_deg, d3_deg = np.degrees(th1), np.degrees(th2), np.degrees(th3)
                if 0 < d1_deg < 90 :
                    P2 = 125 + d1_deg / 90 * (500 - 125)
                    P3 = 500 / 120 * (120 + d2_deg)
                    P4 = 500 + (d3_deg / 120) * 500 if d3_deg >= 0 else 500 * (120 + d3_deg) / 120
                    s = d1_deg + d2_deg + d3_deg
                    if s < best_sum:
                        best_sum, best = s, {"P2":P2,"P3":P3,"P4":P4}
            if not best:
                print("No Answer exists.")
                return
            req = [(3,best["P3"]),(4,best["P4"])]
            for sid, pulse in req:
                pulse = int(round(pulse))
                setBusServoPulse(sid, pulse, 500, ser2)
                time.sleep(0.2)
            time.sleep(0.5)
            setBusServoPulse(2, best["P2"], 1000, ser2)
            time.sleep(1)
            setBusServoPulse(6, 700, 100, ser2)
            time.sleep(0.5)
            print("Control completed")
            time.sleep(1)
            return best["P2"], best["P3"], best["P4"]
        else:
            
            th3 = -np.arccos(cos_val)
            
            #if (l2 -l3)**2 < A**2 + B**2 < (l2 + l3)**2:
            #    print("The solution exists")
            #else:
            #    print("!The solution does not exist, need to move the dog!")
            print("Use later method")
            D = l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3)
            cosine = (A * (l2 + l3 * np.cos(th3)) + B * (l3 * np.sin(th3)))/D
            sine = (-A * (l3 * np.sin(th3)) + B * (l2 + l3 * np.cos(th3)))/D

            th2 = np.arctan2(sine, cosine) - fai

            d1_deg, d2_deg, d3_deg = np.degrees(fai), np.degrees(th2), np.degrees(th3)
            if 0 < d1_deg < 100 :
                P3 = 500/120 * (120 + d2_deg)
                P4 = 500 + (d3_deg/120)*500 if d3_deg>=0 else 500*(120 + d3_deg)/120
            else:
                return []
            req = [(3, P3), (4, P4)]
            
            for sid, pulse in req:
                pulse = int(round(pulse))
                setBusServoPulse(sid, pulse, 500, ser2)
                time.sleep(0.2)
            time.sleep(0.5)
            setBusServoPulse(2, P2, 1000, ser2)
            time.sleep(1.5)
            setBusServoPulse(6, 650, 100,ser2)
            time.sleep(0.5)
            print("Control completed")
            time.sleep(1)
            return P2, P3, P4
    
    def Calculate_Arm_Angle(self, px, py, pz):
        
        x1 = np.hypot(px, py)
        y1 = pz
        P2 = 300
        d1_deg = (P2 - 125) / (500 - 125) * 90
        fai = np.radians(d1_deg)
        
        A = x1 - l1 * np.cos(fai)
        B = y1 - l1 * np.sin(fai)
        #th3 = -np.arccos((A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3))
        
        cos_val = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
        if not np.isfinite(cos_val) or cos_val < -1.0 or cos_val > 1.0:
            sols = self.batch_solve_theta(px, py, pz)
            best, best_sum = None, float('inf')
            for th1, th2, th3 in sols:
                d1_deg, d2_deg, d3_deg = np.degrees(th1), np.degrees(th2), np.degrees(th3)
                if 0 < d1_deg < 90 :
                    P2 = 125 + d1_deg / 90 * (500 - 125)
                    P3 = 500 / 120 * (120 + d2_deg)
                    P4 = 500 + (d3_deg / 120) * 500 if d3_deg >= 0 else 500 * (120 + d3_deg) / 120
                    s = d1_deg + d2_deg + d3_deg
                    if s < best_sum:
                        best_sum, best = s, {"P2":P2,"P3":P3,"P4":P4}
            return best["P2"], best["P3"], best["P4"]
        else:
            th3 = -np.arccos(cos_val)
            
            D = l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3)
            cosine = (A * (l2 + l3 * np.cos(th3)) + B * (l3 * np.sin(th3)))/D
            sine = (-A * (l3 * np.sin(th3)) + B * (l2 + l3 * np.cos(th3)))/D

            th2 = np.arctan2(sine, cosine) - fai

            d1_deg, d2_deg, d3_deg = np.degrees(fai), np.degrees(th2), np.degrees(th3)
            P3 = 500/120 * (120 + d2_deg)
            P4 = 500 + (d3_deg/120)*500 if d3_deg>=0 else 500*(120 + d3_deg)/120
            return P2, P3, P4
    
    def reset_arm(self, P1, P2, P3, P4, P5, P6, ser2):
        reset_seq = [(1, P1), (2, P2), (3, P3), (4, P4), (5, P5)]
        for sid, pulse in reset_seq:
            pulse = round(pulse)     
            setBusServoPulse(sid, pulse, 300, ser2)
            time.sleep(0.2)
        time.sleep(1)
        setBusServoPulse(6, int(P6), 200, ser2)
        
    def reset(self, P1, P2, P3, P4, P5, ser2):
        reset_seq = [(1, P1), (2, P2), (3, P3), (4, P4), (5, P5)]
        for sid, pulse in reset_seq:
            pulse = round(pulse)     
            setBusServoPulse(sid, pulse, 300, ser2)
            time.sleep(0.2)
        
    def arm_catch_and_put(self, ser0, ser1, ser2):
        #Beginning of the robotic arm control sequence
        #self.reset_arm(125, 500, 500, 500, 530, 0, ser2)
        #self.reset_arm(125, 700, 125, 100, 530, 0, ser2)
        setBusServoPulse(6, 0, 100, ser2)
        time.sleep(1)
        #self.reset(125, 500, 500, 500, 530, ser2)
        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #!!!!!!should get back to 125
        self.reset(150, 700, 125, 100, 500, ser2)
        self.reset(500, 600, 0, 400, 500, ser2)
        time.sleep(0.5)
        #self.reset_arm(500, 500, 500, 500, 530, 0)
        #self.reset_arm(400, 500, 125, 300, 530, 0, ser2)
        #self.reset_arm(500, 600, 0, 400, 530, 0, ser2)
       
        # Step 1: Locate the target angle and distance
        P1= self.locate_target_angle(ser0, ser2, baudrate=921600)
        time.sleep(0.2)
        print(P1)
        reset_seq = [ (3, 100), (4, 700)]
        for sid, pulse in reset_seq:
            setBusServoPulse(sid, pulse, 1000, ser2)
        time.sleep(0.2)
        print("Distance")
        dist_mm = self.locate_target_distance(ser1, ser2, baudrate=921600)
        offset = 0.3
        # offset refers to the Distance from the distance sensor to the origin of the robotic arm coordinates
        # Step 2: Construct the end target coordinates (m): lateral offset py=0, depth px=dist_mm/1000, given pz
        px = (dist_mm + offset) / 1000.0
        py = 0.0
        pz = -0.05  # The vertical height difference from the center of the object to the center of servo 2
        # Step 3: Solve and send servo 2~4 pulse widths
        self.control_arm(px, py, pz, ser2, nx=1.0, ny=0.0)
        
        
        print(1)
        ser = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        
        
        
        
        P2, P3, P4 = self.Calculate_Arm_Angle(px+0.03, py, pz = -0.05)
        
        
        
        #print(2)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        
        
        
        #P2, P3, P4 = self.control_arm(px+0.05, py, pz, nx=1.0, ny=0.0)
        #time.sleep(0.5)
        
        # Step 4: Reset the robotic arm
        reset_seq = [(2, 600), (3, 100), (4, 500)]
        for sid, pulse in reset_seq:
            setBusServoPulse(sid, pulse, 1000, ser2)
            time.sleep(0.2)
        time.sleep(0.5)
        # Step 5: Place the object
        #self.reset_arm(125, 550, 100, 500, 530, 800, ser2)
        #time.sleep(0.5)
        #!!!!!!should get back to 125
        self.reset(150, 500, 100, 500, 500, ser2)
        time.sleep(0.5)
        setBusServoPulse(4, 300, 1000, ser2)
        #setBusServoPulse(2, 450, 1000, ser2)
        time.sleep(1)
        setBusServoPulse(6, 0, 100, ser2)
        print(1)
        time.sleep(1)
        
        setBusServoPulse(2, 600, 1000, ser2)
        time.sleep(0.5)
        
        
        
        #print(3)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        
        
        # Step 6: Get Another Object
        #self.reset_arm(900, 500, 125, 300, 530, 0, ser2)
        #time.sleep(0.5)
        self.reset(900, 500, 125, 300, 500, ser2)
        time.sleep(0.5)
        setBusServoPulse(2, 450, 1000, ser2)
        
        #time.sleep(0.5)
        setBusServoPulse(4, 200, 300, ser2)
        time.sleep(1)
        #setBusServoPulse(4, 250, 1000) 
        #self.reset_arm(875, 450, 50, 350, 530, 0)
        setBusServoPulse(6, 650, 100, ser2)
        time.sleep(1)
        setBusServoPulse(2, 700, 500, ser2)
        time.sleep(0.5)
        
        #self.reset_arm(500, 500, 100, 500, 530, 800)
        # Step 7: Put the Object to the right place
        #self.reset_arm(P1, P2, P3, P4, 530, 1000)
        reset_seq = [(1, P1), (3, P3), (4, P4), (2, P2)]
        for sid, pulse in reset_seq:
            pulse = round(pulse)     
            setBusServoPulse(sid, pulse, 500, ser2)
            time.sleep(0.2)
        #reset_seq = [(1, P1), (2, P2), (3, P3), (4, P4) ]
        #for sid, pulse in reset_seq:
        #    pulse = int(round(pulse))        
        #    setBusServoPulse(sid, pulse, 400)
        #    time.sleep(0.2)
        time.sleep(1)
        setBusServoPulse(6, 0, 300, ser2)
        time.sleep(1)
        
        
        print(4)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        #self.reset_arm(500,500,500,500,500,0,ser2)
        #time.sleep(1)
        setBusServoPulse(2, 500, 500, ser2)
        time.sleep(0.1)
        setBusServoPulse(3, 500, 500, ser2)
       
        setBusServoPulse(4, 500, 500, ser2)
        time.sleep(0.5)
        
        
        #self.reset(500, 500, 500, 500, 500, ser2)
        #time.sleep(1)
        
        '''
        
        setBusServoPulse(2, 500, 1000, ser2)
        time.sleep(0.5)
        setBusServoPulse(2, 775, 1000, ser2)
        time.sleep(0.5)
        '''
        #print(5)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        '''
        setBusServoPulse(4, 150, 1000, ser2)
        time.sleep(0.5)
        setBusServoPulse(3, 0, 1000, ser2)
        time.sleep(0.5)
        '''
        #print(6)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        
        #setBusServoPulse(1, 125, 1000, ser2)
        #time.sleep(0.5)
        #self.reset_arm(125, 550, 100, 600, 530, 0, ser2)
        #time.sleep(0.5)
        
        #print(7)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        
        #self.reset_arm(125, 350, 200, 200, 530, 0, ser2)
        #time.sleep(0.5)
        
        #!!!!!!should get back to 125
        setBusServoPulse(1, 150, 100, ser2)
        time.sleep(0.5)
        setBusServoPulse(4, 220, 100, ser2)
        time.sleep(0.2)
        setBusServoPulse(3, 200, 100, ser2)
        time.sleep(0.2)
        setBusServoPulse(2, 380, 500, ser2)
        time.sleep(1)
        setBusServoPulse(6, 700, 100, ser2)
        time.sleep(0.3)
        
        #print(8)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        
        
        self.fin_circle = True
        
    def Arm_put(self, ser0, ser2):
        setBusServoPulse(2, 600, 1000, ser2)
        time.sleep(0.5)
        self.reset_arm(400, 500, 100, 300, 530, 0, ser2)
        # Step 1: Locate the target angle and distance
        P1= self.locate_target_angle(ser0, baudrate=921600)
        #setBusServoPulse(2, 650, 1000)
        #time.sleep(0.5)
        #self.reset_arm(500, 500, 100, 300, 530, 1000)
        #time.sleep(0.5)
        #reset_seq = [ (3, 100), (4, 700)]
        #for sid, pulse in reset_seq:
        #    setBusServoPulse(sid, pulse, 200)
        #dist_mm = self.locate_target_distance(laser_port='/dev/ttyUSB1', baudrate=115200)
        P2, P3, P4 = self.Calculate_Arm_Angle(0.35 , 0.0, 0.0)
        #self.reset_arm(500, 600, 100, 500, 530, 0)
        #time.sleep(0.5)
        #self.reset_arm(125, 550, 100, 500, 530, 0)
        #time.sleep(0.5)
        #setBusServoPulse(2, 450, 1000)
        #setBusServoPulse(4, 300, 1000)
        #time.sleep(1)
        #setBusServoPulse(6, 1000, 1000)
        #time.sleep(1)
        #setBusServoPulse(2, 600, 1000)
        #time.sleep(1)
        #setBusServoPulse(1, 500, 1000)
        #time.sleep(1)
        
        self.reset_arm(P1, P2, P3, P4, 530, 1000, ser2)
        time.sleep(0.5)
        setBusServoPulse(6, 0, 1000, ser2)
        time.sleep(0.5)
        setBusServoPulse(2, 775, 200, ser2)
        setBusServoPulse(4, 150, 200, ser2)
        self.reset_arm(125, 700, 0, 100, 530, 0, ser2)
        self.fin_crossing = True
    
    def Arm_put_optimized(self, ser1, ser2):
        
        setBusServoPulse(2, 500, 200, ser2)
        setBusServoPulse(3, 500, 200, ser2)
        setBusServoPulse(4, 500, 200, ser2)
        time.sleep(0.5)
        setBusServoPulse(1, 500, 200, ser2)
        time.sleep(0.5)
        
        distances_500 = []
        distances_400 = []
        distances_600 = []
        
        for _ in range(20):
            time.sleep(0.1)
            d = read_distance(ser1)
            if d is not None and 100 < d < 450:
                print(f"500 pos: {d}")
                distances_500.append(d)
            time.sleep(0.05)
        
        setBusServoPulse(1, 350, 200, ser2)
        time.sleep(0.5)
        
        for _ in range(20):
            time.sleep(0.1)
            d = read_distance(ser1)
            if d is not None and 100 < d < 450:
                print(f"400 pos: {d}")
                distances_400.append(d)
            time.sleep(0.05)
        
        setBusServoPulse(1, 650, 200, ser2)
        time.sleep(0.5)
        
        for _ in range(20):
            time.sleep(0.1)
            d = read_distance(ser1)
            if d is not None and 100 < d < 450:
                print(f"600 pos: {d}")
                distances_600.append(d)
            time.sleep(0.05)
        
        def process_distances(dist_list):
            if len(dist_list) <= 4:
                return np.var(dist_list) if dist_list else float('inf')
            sorted_list = sorted(dist_list)
            trimmed_list = sorted_list[2:-2]
            return np.mean(trimmed_list)
        
        var_500 = process_distances(distances_500)
        var_400 = process_distances(distances_400)
        var_600 = process_distances(distances_600)
        
        print(f"Variances - 500: {var_500:.2f}, 400: {var_400:.2f}, 600: {var_600:.2f}")
        
        if var_500 <= var_400 and var_500 <= var_600:
            target_pos = 500
        elif var_400 <= var_500 and var_400 <= var_600:
            target_pos = 350
        else:
            target_pos = 650
        
        setBusServoPulse(1, target_pos, 200, ser2)
        print(f"Selected position: {target_pos}")
        time.sleep(0.5)
        P2, P3, P4 = self.Calculate_Arm_Angle(px = 0.35, py = 0, pz = -0.04)
        
        
        reset_seq = [(3, P3), (4, P4)]
        for sid, pulse in reset_seq:
            pulse = round(pulse)  
            setBusServoPulse(sid, pulse, 200, ser2)
            time.sleep(0.2)
        time.sleep(0.2)
        setBusServoPulse(2, P2, 1000, ser2)
        time.sleep(0.5)
        setBusServoPulse(6, 0, 100, ser2)
        time.sleep(1)
        setBusServoPulse(2, 600, 1000, ser2)
        time.sleep(0.5)
        self.reset_arm(125, 700, 0, 100, 530, 0, ser2)
        time.sleep(0.5)
        
        #ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
        
        
        
        #P1 = self.locate_target_angle_for_circle(ser1, ser2, baudrate=921600)
        P2, P3, P4 = self.Calculate_Arm_Angle(px = 0.30, py = 0, pz = -0.04)
        
        
        #print(2)
        #ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
        #ser = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
        
        
        self.fin_crossing = True
    
    def arm_put_fixed(self, ser2):
        setBusServoPulse(2, 500, 1000, ser2)
        time.sleep(1)
        setBusServoPulse(3, 500, 800, ser2)
        time.sleep(1)
        setBusServoPulse(4, 500, 800, ser2)
        time.sleep(0.5)
        setBusServoPulse(1, 400, 200, ser2)
        time.sleep(0.5)
        P2, P3, P4 = self.Calculate_Arm_Angle(px = 0.36, py = 0, pz = -0.05)
        
        reset_seq = [(3, P3), (4, P4)]
        for sid, pulse in reset_seq:
            pulse = round(pulse)  
            setBusServoPulse(sid, pulse, 200, ser2)
            time.sleep(0.5)
        time.sleep(0.2)
        setBusServoPulse(2, int(P2), 400, ser2)
        time.sleep(0.5)
        setBusServoPulse(6, 0, 100, ser2)
        time.sleep(1.5)
        setBusServoPulse(2, 600, 1000, ser2)
        time.sleep(0.5)
        self.reset_arm(125, 700, 0, 100, 500, 0, ser2)
        time.sleep(0.5)
        self.fin_crossing = True

if __name__ == "__main__":
    armcontrol = ArmControl()
    '''
    ser1 = serial.Serial('/dev/ttyCH343USB1', 115200, timeout=1)
    while(1):
        d1 = read_distance1(ser1) 
        d2 = read_distance(ser1) 
        print(f"distance1={d1}")
        print(f"distance0={d2}")
    '''
    #armcontrol.Arm_put_optimized(ser1, ser2)
    print(1)
    armcontrol.arm_catch_and_put(ser0,ser1,ser2)
    time.sleep(1)
    print("Finished!")
    armcontrol.arm_put_fixed(ser2)
    ser1 = serial.Serial('/dev/ttyCH343USB1', 115200, timeout=1)
    while(1):
        d1 = read_distance1(ser1) 
        d2 = read_distance(ser1) 
        print(f"distance1={d1}")
        print(f"distance0={d2}")
        
    armcontrol.arm_catch_and_put(ser0,ser1,ser2)
    armcontrol.arm_put_fixed(ser2)
    
    

