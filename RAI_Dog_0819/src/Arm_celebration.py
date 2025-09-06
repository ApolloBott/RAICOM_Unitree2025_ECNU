from BusServoCmd import *
import Board
ser2 = serial.Serial('/dev/arm', 115200, timeout=1)

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

def main():
    from ArmControl import ArmControl
    armcontrol = ArmControl()
    ser2 = serial.Serial('/dev/arm', 115200, timeout=1)
    armcontrol.reset_arm(165, 875, 0, 150, 500, 800, ser2)
    for i in range(1, 20):
        setBusServoPulse(1, i * 20 , 800, ser2)
        time.sleep(0.02)
        setBusServoPulse(4, 450, 1000, ser2)
        time.sleep(0.5)
        setBusServoPulse(4, 150, 1000, ser2)
        time.sleep(0.5)

if __name__ == "__main__":
    main()
