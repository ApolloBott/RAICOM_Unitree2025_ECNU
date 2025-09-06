#change all the serial port to ser0 and ser1

import time
import serial
import serial.tools.list_ports
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

ser2 = serial.Serial('/dev/arm', 115200, timeout=1)

class SerialManager:
    """串口管理器，用于处理串口重连和错误恢复"""
    def __init__(self, port_path, baudrate, timeout=1):
        self.port_path = port_path
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_obj = None
        self.connect()
    
    def connect(self):
        try:
            if self.serial_obj and self.serial_obj.is_open:
                self.serial_obj.close()
            self.serial_obj = serial.Serial(self.port_path, self.baudrate, timeout=self.timeout)
            print(f"[INFO] Successfully connected to {self.port_path}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect to {self.port_path}: {e}")
            return False
    
    def reconnect(self, max_retries=3):
        for attempt in range(max_retries):
            print(f"[INFO] Attempting to reconnect to {self.port_path} (attempt {attempt + 1}/{max_retries})")
            if self.connect():
                return True
            time.sleep(0.5)
        return False
    
    def get_serial(self):
        if not self.serial_obj or not self.serial_obj.is_open:
            if not self.reconnect():
                raise serial.SerialException(f"Unable to maintain connection to {self.port_path}")
        return self.serial_obj

# 创建全局串口管理器
arm_serial_manager = SerialManager('/dev/arm', 115200, timeout=1)


def setBusServoPulse(id, pulse, use_time, ser2, max_retries=3):
    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    
    for attempt in range(max_retries):
        try:
            serial_serro_wirte_cmd(ser2, id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)
            return pulse
        except (serial.SerialException, OSError) as e:
            print(f"[WARNING] Serial write failed on attempt {attempt + 1}/{max_retries}: {e}")
            if attempt < max_retries - 1:
                time.sleep(0.1)  # 短暂等待后重试
                # 尝试重新初始化串口连接
                try:
                    if ser2.is_open:
                        ser2.close()
                    time.sleep(0.1)
                    ser2.open()
                    print(f"[INFO] Serial port reopened successfully")
                except Exception as reopen_error:
                    print(f"[ERROR] Failed to reopen serial port: {reopen_error}")
            else:
                print(f"[ERROR] All {max_retries} attempts failed for servo {id}")
                raise e
    return pulse
    
class ArmControl:
    def __init__(self):
        self.fin_circle = False
        self.fin_crossing = False

    def check_serial(self, port_name: str, baudrate: int, timeout: float = 1.0):
        try:
            ser = serial.Serial(port_name, baudrate, timeout=timeout)
            if ser.is_open:
                print(f"[OK] {port_name} is open")
                return ser
        except serial.SerialException as e:
            print(f"[ERROR] can't open {port_name}, error: {e}")
        except ValueError as e:
            print(f"[ERROR] invalid port value for {port_name}: {e}")
        return None

    def reconnect_serial(self, ser, port_name: str, baudrate: int, timeout: float = 1.0, max_retries=3):
        """
        重连串口的方法
        """
        for attempt in range(max_retries):
            try:
                if ser and ser.is_open:
                    ser.close()
                time.sleep(0.2)  # 等待串口释放
                
                new_ser = serial.Serial(port_name, baudrate, timeout=timeout)
                if new_ser.is_open:
                    print(f"[INFO] Successfully reconnected to {port_name} on attempt {attempt + 1}")
                    return new_ser
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Reconnect attempt {attempt + 1}/{max_retries} failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(0.5)  # 等待更长时间再重试
        
        print(f"[ERROR] Failed to reconnect to {port_name} after {max_retries} attempts")
        return None

    def safe_servo_control(self, id, pulse, use_time, ser2, port_name='/dev/arm', baudrate=115200):
        """
        安全的舵机控制方法，包含重连机制
        """
        max_retries = 3
        for attempt in range(max_retries):
            try:
                return setBusServoPulse(id, pulse, use_time, ser2, max_retries=2)
            except (serial.SerialException, OSError) as e:
                print(f"[ERROR] Servo control failed: {e}")
                if attempt < max_retries - 1:
                    print(f"[INFO] Attempting to reconnect serial port...")
                    new_ser = self.reconnect_serial(ser2, port_name, baudrate)
                    if new_ser:
                        ser2 = new_ser
                        # 更新全局串口对象
                        globals()['ser2'] = new_ser
                    else:
                        print(f"[ERROR] Failed to reconnect, skipping servo {id} control")
                        return None
                else:
                    print(f"[ERROR] All attempts failed for servo {id}")
                    return None
        return None

    def scan_range(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.1):
        # 原始扫描逻辑（保留）
        # best_pulse, best_dist = pulse_start, float('inf')
        # for pulse in range(pulse_start, pulse_end + 1, pulse_step):
        #     setBusServoPulse(1, pulse, use_time, ser2)
        #     readings = []
        #     for _ in range(6):
        #         d = read_distance(serial_port)  
        #         print(f"raw:{d}")
        #         if d is not None:
        #             readings.append(d)
        #         #time.sleep(0.05)
        #     if readings:
        #         filtered_d = np.mean(readings)
        #         print(filtered_d)
        #         print(np.var(readings))
        #         if filtered_d < best_dist and filtered_d >50 and np.var(readings) < 10:
        #             best_dist, best_pulse = filtered_d, pulse
        # return best_pulse, best_dist
        
        # 优化后的扫描逻辑 - 模仿ArmControl_new
        best_pulse, best_dist = pulse_start, float('inf')
        for pulse in range(pulse_start, pulse_end + 1, pulse_step):
            # 设置舵机位置
            setBusServoPulse(1, pulse, use_time, ser2)
            time.sleep(0.05)  # 短暂等待舵机移动
            
            # 读取距离 - 减少读取次数，提高速度
            readings = []
            for _ in range(4):  # 从6次减少到4次
                d = read_distance(serial_port)
                if d is not None:
                    readings.append(d)
                time.sleep(0.02)  # 减少等待时间
                    
            # 如果有有效读数，计算平均值
            if readings:
                filtered_d = np.mean(readings)
                print(f"脉冲 {pulse}: {filtered_d:.1f}mm (方差: {np.var(readings):.1f})")
                
                # 优化筛选条件
                if (filtered_d < best_dist and 
                    filtered_d > 50 and 
                    filtered_d < 1000 and  # 添加上限
                    np.var(readings) < 15):  # 稍微放宽方差要求
                    best_dist, best_pulse = filtered_d, pulse
                    
        return best_pulse, best_dist

    # def scan_range_short(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.2):
        
    #     best_pulse, best_dist = pulse_start, float('inf')
    #     for pulse in range(pulse_start, pulse_end + 1, pulse_step):
    #         setBusServoPulse(1, pulse, use_time, ser2)
    #         time.sleep(delay)
    #         #Change to New Laser Detect module
    #         d = read_distance(serial_port)  
    #         print(d)
    #         if d is not None and d < 200:
    #             if d < best_dist and d > 50:
    #                 best_dist, best_pulse = d, pulse
    #     return best_pulse, best_dist

    # def scan_range_for_circle(self, serial_port, ser2, pulse_start, pulse_end, pulse_step, use_time=200, delay=0.15):
    #     best_pulse, best_dist = pulse_start, float('inf')
        
    #     for pulse in range(pulse_start, pulse_end + 1, pulse_step):
    #         setBusServoPulse(1, pulse, use_time, ser2)
    #         readings = []
    #         read_count = 0
    #         while len(readings) < 15:
    #             d = read_distance(serial_port)
    #             read_count += 1
    #             if read_count <= 5:
    #                 continue
    #             if 300 < d < 500:
    #                 readings.append(d)
    #             time.sleep(0.02)
    #         filtered_d = np.mean(readings)
    #         print(f"Pulse: {pulse}mm, Distance: {filtered_d:.1f}mm")
            
    #         best_dist, best_pulse = filtered_d, pulse
    #         break  
        
    #     return best_dist, best_pulse
    
 
    def locate_target_angle(self, ser0, ser2, baudrate=921600):
        """
        Locate the target angle using a laser distance sensor.
        Output:
        Returns the best pulse width for servo 1.
        """
        #ser = serial.Serial(laser_port, baudrate, timeout=1)
        #time.sleep(0.02)
       
        cp, cd = self.scan_range(ser0, ser2, 350, 600, 5)
        #time.sleep(0.1)
        fs, fe = max(300, cp-5), min(700, cp+5)
        
        fp, fd = self.scan_range(ser0, ser2, fs, fe, 1)
        time.sleep(0.1)
        if 200 <= fp <= 800:
            setBusServoPulse(1, fp, 1000, ser2)
            time.sleep(0.15)
            #ser.close()
            return fp  
  

    def locate_target_distance(self, ser1, ser2, baudrate=115200):
        # 原始距离定位逻辑（保留）
        # time.sleep(0.1)
        # distances = []
        # for _ in range(8):
        #     try:
        #         time.sleep(0.05)
        #         ser1 = serial.Serial('/dev/detector1', 115200, timeout=1)
        #         d = read_distance1(ser1)         
        #         if d is not None and 100 < d < 900:
        #             print(f"distance = {d}")
        #             distances.append(d)
        #     except (IndexError, serial.SerialException) as e:
        #         print(f"Fail{e},Use 270")
        #         distances.append(250)
        #     finally:
        #         if ser1 and ser1.is_open:
        #             ser1.close()
        #     time.sleep(0.05)
        # if len(distances) == 0:
        #     return 250
        # avg_dist = np.mean(distances)
        # print(f"average distance is: {avg_dist}")
        # return avg_dist
        
        # 优化后的距离定位逻辑 - 模仿ArmControl_new
        distances = []
        valid_readings = 0
        max_attempts = 5  # 减少尝试次数
        start_time = time.time()
        max_time = 2.0  # 最多2秒超时

        # 确保串口是打开的
        if not ser1 or not ser1.is_open:
            try:
                print("重新打开串口 /dev/detector1")
                ser1 = serial.Serial('/dev/detector1', 115200, timeout=1)
            except Exception as e:
                print(f"无法打开串口: {e}")
                return 250

        # 清空缓冲区
        try:
            ser1.reset_input_buffer()
        except Exception as e:
            print(f"清空缓冲区失败: {e}")
        
        # 读取距离数据 - 添加总体超时保护
        print("开始读取距离数据...")
        while valid_readings < 3 and len(distances) < max_attempts and (time.time() - start_time < max_time):
            try:
                d = read_distance1(ser1)
                if d is not None and 100 < d < 900:
                    distances.append(d)
                    valid_readings += 1
                    print(f"有效读取 #{valid_readings}: {d}mm")
                else:
                    print(f"无效读取: {d}")
            except Exception as e:
                print(f"读取失败: {e}")
            
            time.sleep(0.05)

        # 检查是否超时
        if time.time() - start_time >= max_time:
            print("读取超时，使用可用数据或默认值")
        
        # 如果没有有效读数，返回默认值
        if not distances:
            print("未获取有效距离，使用默认值")
            return 250

        # 计算平均值
        avg_dist = sum(distances) / len(distances)
        print(f"目标距离: {avg_dist:.1f}mm")
        return avg_dist

    def batch_solve_theta(self, px, py, pz, th3_min=-120, th3_max=120, th3_step=1):
        # 原始批量求解逻辑（保留）
        # '''
        # Figure out the ik of the 6d Robotic arm
        # '''
        # solutions = []
        # x1 = (px**2 + py**2)**0.5  
        # y1 = pz                
        # for th3_deg in range(th3_min, th3_max+1, th3_step):
        #     th3 = np.deg2rad(th3_deg)
        #     A = 2 * l1 * (l2 + l3 * np.cos(th3))
        #     B = -2 * l1 * l3 * np.sin(th3)
        #     C = x1**2 + y1**2 - (l1**2 + l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3))
        #     R = np.hypot(A, B)
        #     if abs(C / R) <= 1:
        #         phi   = np.arctan2(B, A)
        #         delta = np.arccos(C / R)
        #         for sign in (+1, -1):
        #             th2 = phi + sign * delta
        #             A_ = l1 + l2 * np.cos(th2) + l3 * np.cos(th2 + th3)
        #             B_ =       l2 * np.sin(th2) + l3 * np.sin(th2 + th3)
        #             th1 = np.arctan2(A_ * y1 - B_ * x1, A_ * x1 + B_ * y1)
        #             solutions.append((th1, th2, th3))
        # return solutions
        
        '''
        Figure out the ik of the 6d Robotic arm
        优化版本: 先使用较大步长搜索，再在有解区域进行精细搜索
        '''
        solutions = []
        x1 = (px**2 + py**2)**0.5  
        y1 = pz
        
        # 先用较大步长找到可能有解的区域
        potential_regions = []
        coarse_step = 5  # 粗搜索步长
        
        for th3_deg in range(th3_min, th3_max+1, coarse_step):
            th3 = np.deg2rad(th3_deg)
            A = 2 * l1 * (l2 + l3 * np.cos(th3))
            B = -2 * l1 * l3 * np.sin(th3)
            C = x1**2 + y1**2 - (l1**2 + l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3))
            R = np.hypot(A, B)
            if abs(C / R) <= 1:
                potential_regions.append(th3_deg)
        
        # 如果粗搜索没找到可能的区域，扩大搜索范围
        if not potential_regions:
            print("粗搜索未找到解，扩大搜索范围")
            extended_min = max(th3_min - 20, -180)
            extended_max = min(th3_max + 20, 180)
            for th3_deg in range(extended_min, extended_max+1, coarse_step):
                th3 = np.deg2rad(th3_deg)
                A = 2 * l1 * (l2 + l3 * np.cos(th3))
                B = -2 * l1 * l3 * np.sin(th3)
                C = x1**2 + y1**2 - (l1**2 + l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3))
                R = np.hypot(A, B)
                if abs(C / R) <= 1:
                    potential_regions.append(th3_deg)
        
        # 在有希望的区域进行精细搜索
        regions_to_search = []
        for center in potential_regions:
            start = max(th3_min, center - coarse_step)
            end = min(th3_max, center + coarse_step)
            regions_to_search.append((start, end))
        
        # 如果仍然没有找到区域，就搜索整个范围
        if not regions_to_search:
            regions_to_search = [(th3_min, th3_max)]
        
        # 在每个区域进行精细搜索
        for start, end in regions_to_search:
            for th3_deg in range(start, end+1, th3_step):
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
        
        if not solutions:
            print("未找到解决方案，尝试更宽松的约束条件")
            # 如果还是没有解，尝试使用更宽松的条件
            for th3_deg in range(th3_min, th3_max+1, th3_step):
                th3 = np.deg2rad(th3_deg)
                A = 2 * l1 * (l2 + l3 * np.cos(th3))
                B = -2 * l1 * l3 * np.sin(th3)
                C = x1**2 + y1**2 - (l1**2 + l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3))
                R = np.hypot(A, B)
                if R > 0 and abs(C / R) <= 1.1:  # 稍微放宽条件
                    phi   = np.arctan2(B, A)
                    delta = np.arccos(max(-1, min(1, C / R)))  # 确保在有效范围内
                    for sign in (+1, -1):
                        th2 = phi + sign * delta
                        A_ = l1 + l2 * np.cos(th2) + l3 * np.cos(th2 + th3)
                        B_ =       l2 * np.sin(th2) + l3 * np.sin(th2 + th3)
                        th1 = np.arctan2(A_ * y1 - B_ * x1, A_ * x1 + B_ * y1)
                        solutions.append((th1, th2, th3))
                        
        return solutions
    
    def control_arm(self, px, py, pz, ser2, nx = 1.0, ny = 0, th3_min=-120, th3_max=120, th3_step=1):
        # 原始控制逻辑（保留）
        # pz = -0.05 
        # x1 = np.hypot(px, py)
        # y1 = pz
        # if px < 0.2:
        #     P2 = 500
        # elif 0.2 <= px < 0.27:
        #     P2 = 400
        # elif 0.27 <= px < 0.32:
        #     P2 = 350
        # elif 0.32 <= px < 0.36:
        #     P2 = 300
        # else:
        #     P2 = 200
        # #P2 = -1123.5059 * (np.hypot(px,pz)) + 715.6031
        # d1_deg = (P2 - 125) / (500 - 125) * 90
        # fai = np.radians(d1_deg)
        
        # 优化后的控制逻辑 - 模仿ArmControl_new
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
            setBusServoPulse(6, 700, 700, ser2)
            time.sleep(0.5)
            print("Control completed")
            time.sleep(1)
            return best["P2"], best["P3"], best["P4"]  # 返回计算的值
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
            print(f"d1_deg={d1_deg},d2_deg={d2_deg},d3_deg={d3_deg}")
            if 0 < d1_deg < 100 :
                P3 = 500/120 * (120 + d2_deg)
                P4 = 500 + (d3_deg/120)*500 if d3_deg>=0 else 500*(120 + d3_deg)/120
            else:
                return []
            req = [(3, P3), (4, P4)]
            
            for sid, pulse in req:
                pulse = int(round(pulse))
                try:
                    setBusServoPulse(sid, pulse, 500, ser2)
                    time.sleep(0.2)
                except (serial.SerialException, OSError) as e:
                    print(f"[WARNING] Failed to control servo {sid} in control_arm: {e}")
                    continue
            time.sleep(0.5)
            try:
                setBusServoPulse(2, P2, 1000, ser2)
                time.sleep(1)
                setBusServoPulse(6, 700, 700, ser2)
                time.sleep(1)
                print("Control completed")
                time.sleep(1)
                return P2, P3, P4
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Failed to complete final servo control in control_arm: {e}")
                return P2, P3, P4
    
    def Calculate_Arm_Angle(self, px, py, pz):
        # 原始角度计算逻辑（保留）
        # x1 = np.hypot(px, py)
        # y1 = pz
        # P2 = 300
        # d1_deg = (P2 - 125) / (500 - 125) * 90
        # fai = np.radians(d1_deg)
        # A = x1 - l1 * np.cos(fai)
        # B = y1 - l1 * np.sin(fai)
        # cos_val = (A**2 + B**2 - l2**2 - l3**2) / (2 * l2 * l3)
        # if not np.isfinite(cos_val) or cos_val < -1.0 or cos_val > 1.0:
        #     sols = self.batch_solve_theta(px, py, pz)
        #     best, best_sum = None, float('inf')
        #     for th1, th2, th3 in sols:
        #         d1_deg, d2_deg, d3_deg = np.degrees(th1), np.degrees(th2), np.degrees(th3)
        #         if 0 < d1_deg < 90 :
        #             P2 = 125 + d1_deg / 90 * (500 - 125)
        #             P3 = 500 / 120 * (120 + d2_deg)
        #             P4 = 500 + (d3_deg / 120) * 500 if d3_deg >= 0 else 500 * (120 + d3_deg) / 120
        #             s = d1_deg + d2_deg + d3_deg
        #             if s < best_sum:
        #                 best_sum, best = s, {"P2":P2,"P3":P3,"P4":P4}
        #     return best["P2"], best["P3"], best["P4"]
        # else:
        #     th3 = -np.arccos(cos_val)
        #     D = l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3)
        #     cosine = (A * (l2 + l3 * np.cos(th3)) + B * (l3 * np.sin(th3)))/D
        #     sine = (-A * (l3 * np.sin(th3)) + B * (l2 + l3 * np.cos(th3)))/D
        #     th2 = np.arctan2(sine, cosine) - fai
        #     d1_deg, d2_deg, d3_deg = np.degrees(fai), np.degrees(th2), np.degrees(th3)
        #     P3 = 500/120 * (120 + d2_deg)
        #     P4 = 500 + (d3_deg/120)*500 if d3_deg>=0 else 500*(120 + d3_deg)/120
        #     return P2, P3, P4
        
        # 优化后的角度计算逻辑 - 模仿ArmControl_new
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
            print("使用批量求解方法计算角度")
            sols = self.batch_solve_theta(px, py, pz)
            best, best_sum = None, float('inf')
            for th1, th2, th3 in sols:
                d1_deg, d2_deg, d3_deg = np.degrees(th1), np.degrees(th2), np.degrees(th3)
                if 0 < d1_deg < 90 :
                    P2_calc = 125 + d1_deg / 90 * (500 - 125)
                    P3_calc = 500 / 120 * (120 + d2_deg)
                    P4_calc = 500 + (d3_deg / 120) * 500 if d3_deg >= 0 else 500 * (120 + d3_deg) / 120
                    s = d1_deg + d2_deg + d3_deg
                    if s < best_sum:
                        best_sum, best = s, {"P2":P2_calc,"P3":P3_calc,"P4":P4_calc}
            
            if best:
                return best["P2"], best["P3"], best["P4"]
            else:
                print("未找到有效解，返回默认值")
                return P2, 500, 500
        else:
            print("使用直接计算方法")
            th3 = -np.arccos(cos_val)
            
            D = l2**2 + l3**2 + 2 * l2 * l3 * np.cos(th3)
            cosine = (A * (l2 + l3 * np.cos(th3)) + B * (l3 * np.sin(th3)))/D
            sine = (-A * (l3 * np.sin(th3)) + B * (l2 + l3 * np.cos(th3)))/D

            th2 = np.arctan2(sine, cosine) - fai

            d1_deg, d2_deg, d3_deg = np.degrees(fai), np.degrees(th2), np.degrees(th3)
            P3 = 500/120 * (120 + d2_deg)
            P4 = 500 + (d3_deg/120)*500 if d3_deg>=0 else 500*(120 + d3_deg)/120
            
            print(f"计算结果: P2={P2}, P3={P3:.1f}, P4={P4:.1f}")
            return P2, P3, P4
    
    def reset_arm(self, P1, P2, P3, P4, P5, P6, ser2):
        reset_seq = [(1, P1), (2, P2), (3, P3), (4, P4), (5, P5)]
        for sid, pulse in reset_seq:
            pulse = round(pulse)
            try:
                setBusServoPulse(sid, pulse, 500, ser2)
                time.sleep(0.2)
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Failed to set servo {sid} to {pulse}: {e}")
                continue  # 继续尝试其他舵机
        time.sleep(0.5)
        try:
            setBusServoPulse(6, int(P6), 500, ser2)
        except (serial.SerialException, OSError) as e:
            print(f"[WARNING] Failed to set servo 6 to {P6}: {e}")
        
    def reset(self, P1, P2, P3, P4, P5, ser2):
        reset_seq = [(1, P1), (2, P2), (3, P3), (4, P4), (5, P5)]
        for sid, pulse in reset_seq:
            pulse = round(pulse)
            try:
                setBusServoPulse(sid, pulse, 500, ser2)
                time.sleep(0.2)
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Failed to set servo {sid} to {pulse}: {e}")
                continue  # 继续尝试其他舵机
        
    def arm_catch_and_put(self, ser0, ser1, ser2):
        """机械臂抓取和放置操作，现在包含错误处理"""
        try:
            print("LuoTianHao")
            #Beginning of the robotic arm control sequence
            setBusServoPulse(6, 0, 100, ser2)
            time.sleep(0.1)
            #!!!!!!should get back to 125
            #self.reset(150, 700, 125, 100, 500, ser2)
            self.reset(500, 600, 0, 400, 500, ser2)
            time.sleep(0.1)
           
            # Step 1: Locate the target angle and distance
            print("Step 1: Locate the target angle and distance")
            P1= self.locate_target_angle(ser0, ser2, baudrate=921600)
            time.sleep(0.2)
            print(P1)
            reset_seq = [ (3, 100), (4, 700)]
            for sid, pulse in reset_seq:
                try:
                    setBusServoPulse(sid, pulse, 1000, ser2)
                except (serial.SerialException, OSError) as e:
                    print(f"[WARNING] Failed to set servo {sid} in step 1: {e}")
            time.sleep(0.2)
            print("Distance")
            dist_mm = self.locate_target_distance(ser1, ser2, baudrate=921600)
            offset = 3
            # offset refers to the Distance from the distance sensor to the origin of the robotic arm coordinates



            # Step 2: Construct the end target coordinates (m): lateral offset py=0, depth px=dist_mm/1000, given pz
            print("Step 2: Construct the end target coordinates (m): lateral offset py=0, depth px=dist_mm/1000, given pz")
            px = (dist_mm + offset) / 1000.0
            py = 0.0
            pz = -0.05  # The vertical height difference from the center of the object to the center of servo 2



            # Step 3: Solve and send servo 2~4 pulse widths and catch the object
            print("Step 3: Solve and send servo 2~4 pulse widths and catch the object")
            self.control_arm(px, py, pz, ser2, nx=1.0, ny=0.0)
            print(1)
            ser0 = serial.Serial('/dev/detector0', 921600, timeout=1)
            P2, P3, P4 = self.Calculate_Arm_Angle(px+0.03, py, pz = -0.05)
            
            
            # Step 4: Reset the robotic arm
            print("Step 4: Reset the robotic arm")
            reset_seq = [(2, 600), (3, 100), (4, 500)]
            for sid, pulse in reset_seq:
                try:
                    setBusServoPulse(sid, pulse, 1000, ser2)
                    time.sleep(0.2)
                except (serial.SerialException, OSError) as e:
                    print(f"[WARNING] Failed to set servo {sid} in step 4: {e}")
            time.sleep(0.1)



            # Step 5: Place the object to the front basket
            #!!!!!!should get back to 125
            print("Step 5: Place the object to the front basket")
            self.reset(150, 500, 100, 500, 500, ser2)
            time.sleep(0.2)
            try:
                setBusServoPulse(4, 330, 1000, ser2)
                time.sleep(1)
                setBusServoPulse(6, 0, 500, ser2)
                time.sleep(1)
                setBusServoPulse(2, 600, 1000, ser2)
                time.sleep(0.2)
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Error in step 5: {e}")
            


            # Step 6: Get Another Object
            print("Step 6: Get Another Object")
            self.reset(875, 500, 125, 300, 500, ser2)
            time.sleep(0.5)
            try:
                setBusServoPulse(2, 450, 1000, ser2)
                setBusServoPulse(4, 200, 300, ser2)
                time.sleep(0.5)
                setBusServoPulse(6, 650, 700, ser2)
                time.sleep(1)
                setBusServoPulse(2, 700, 500, ser2)
                time.sleep(0.2)
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Error in step 6: {e}")
            


            # Step 7: Put the Object to the right place
            print("Step 7: Put the Object to the right place")
            reset_seq = [(1, P1), (3, P3), (4, P4), (2, P2)]
            for sid, pulse in reset_seq:
                pulse = round(pulse)
                try:
                    setBusServoPulse(sid, pulse, 1000, ser2)
                    time.sleep(0.2)
                except (serial.SerialException, OSError) as e:
                    print(f"[WARNING] Failed to set servo {sid} in step 7: {e}")
            time.sleep(1)
            try:
                setBusServoPulse(6, 0, 500, ser2)
                time.sleep(1)
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Error setting servo 6 in step 7: {e}")
            


            # Step 8: Reset the arm
            print("Step 8: Reset the arm")
            try:
                setBusServoPulse(2, 500, 500, ser2)
                time.sleep(0.1)
                setBusServoPulse(3, 500, 500, ser2)
                setBusServoPulse(4, 500, 500, ser2)
                time.sleep(0.5)
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Error in step 8: {e}")




            # Step 9: Catch the Objet and Finish the task
            print("Step 9: Catch the Objet and Finish the task")
            #!!!!!!should get back to 125
            try:
                setBusServoPulse(1, 150, 100, ser2)
                time.sleep(0.5)
                #setBusServoPulse(4, 220, 100, ser2)
                setBusServoPulse(4, 240, 500, ser2)
                time.sleep(0.2)
                setBusServoPulse(3, 220, 500, ser2)
                time.sleep(0.2)
                #setBusServoPulse(2, 380, 500, ser2)
                #setBusServoPulse(2, 420, 1000, ser2)
                setBusServoPulse(2, 320, 1000, ser2)
                time.sleep(1)
                # setBusServoPulse(6, 700, 100, ser2)
                # time.sleep(0.1)
            except (serial.SerialException, OSError) as e:
                print(f"[WARNING] Error in step 9: {e}")
            
            self.fin_circle = True
            print("[INFO] arm_catch_and_put completed successfully")
            
        except Exception as e:
            print(f"[ERROR] Critical error in arm_catch_and_put: {e}")
            print("[INFO] Marking circle task as completed to continue execution")
            self.fin_circle = True
        
    def arm_put_fixed(self, ser2):
        try:
            setBusServoPulse(6, 750, 700, ser2)
            time.sleep(1.3)
            setBusServoPulse(2, 500, 1000, ser2)
            time.sleep(1)
            setBusServoPulse(3, 500, 800, ser2)
            time.sleep(0.5)
            setBusServoPulse(4, 500, 800, ser2)
            time.sleep(0.2)
            setBusServoPulse(1, 400, 200, ser2)
            time.sleep(0.2)
            P2, P3, P4 = self.Calculate_Arm_Angle(px = 0.36, py = 0, pz = -0.05)
            
            reset_seq = [(3, P3), (4, P4)]
            for sid, pulse in reset_seq:
                pulse = round(pulse)  
                setBusServoPulse(sid, pulse, 200, ser2)
                time.sleep(0.5) 
            time.sleep(0.2)
            setBusServoPulse(2, int(P2), 1000, ser2)
            time.sleep(1)
            setBusServoPulse(6, 100, 700, ser2)
            time.sleep(1)
            setBusServoPulse(2, 250, 1000, ser2)
            time.sleep(0.5)
            setBusServoPulse(2, 400, 1000, ser2)
            time.sleep(1)
            
            # setBusServoPulse(3, 450, 1000, ser2)
            # time.sleep(0.5)
            # setBusServoPulse(4, 0, 1000, ser2)
            # time.sleep(0.5)
            # setBusServoPulse(2, 250, 1000, ser2)
            # time.sleep(0.5)
            # setBusServoPulse(3, 500, 1000, ser2)
            # time.sleep(1)
            
            # setBusServoPulse(6, 0, 200, ser2)
            # time.sleep(1)
            self.reset(125, 850, 0, 100, 500, ser2)
            
            self.fin_crossing = True
            
        except (serial.SerialException, OSError) as e:
            print(f"[ERROR] Serial communication error in arm_put_fixed: {e}")
            print("[INFO] Attempting to recover and continue...")
            # 尝试重新连接串口
            try:
                if ser2 and ser2.is_open:
                    ser2.close()
                time.sleep(0.5)
                new_ser2 = serial.Serial('/dev/arm', 115200, timeout=1)
                if new_ser2.is_open:
                    print("[INFO] Serial port reconnected successfully")
                    # 尝试完成关键的复位操作
                    try:
                        self.reset(125, 850, 0, 100, 500, new_ser2)
                        self.fin_crossing = True
                        print("[INFO] Successfully completed reset operation after reconnection")
                    except Exception as reset_error:
                        print(f"[WARNING] Reset operation failed after reconnection: {reset_error}")
                        self.fin_crossing = True  # 仍然标记为完成，避免程序卡住
                else:
                    print("[ERROR] Failed to reconnect serial port")
                    self.fin_crossing = True  # 标记为完成，避免程序卡住
            except Exception as reconnect_error:
                print(f"[ERROR] Failed to handle serial error: {reconnect_error}")
                self.fin_crossing = True  # 标记为完成，避免程序卡住

if __name__ == "__main__":
    ser0 = serial.Serial('/dev/detector0', 921600, timeout=1)
    ser1 = serial.Serial('/dev/detector1', 115200, timeout=1)

    #ser2   = serial.Serial('/dev/ttyUSB0',         115200, timeout=1)   # 原 ttyUSB0 (1-2.1)
    print("LUO")
    

    # while(1):
    #     d1 = read_distance1(ser1) 
    #     d2 = read_distance(ser0) 
    #     print(f"distance1={d1}")
    #     print(f"distance0={d2}")
    
    armcontrol = ArmControl()
    #armcontrol.reset(500, 500, 500, 500, 500, ser2)
    # i = 0 
    # while(i<10):
    #     d1 = read_distance1(ser1) 
    #     i+=1
    #     print(f"distance1={d1}")
        
    # armcontrol.arm_put_fixed(ser2)
    armcontrol.arm_catch_and_put(ser0,ser1,ser2)
    print("Finished!")
    time.sleep(5)
    armcontrol.arm_put_fixed(ser2)
    ser1 = serial.Serial('/dev/detector1', 115200, timeout=1)
    while(1):
        d1 = read_distance1(ser1) 
        d2 = read_distance(ser1) 
        print(f"distance1={d1}")
        print(f"distance0={d2}")
        
    armcontrol.arm_catch_and_put(ser0,ser1,ser2)
    armcontrol.arm_put_fixed(ser2)
    