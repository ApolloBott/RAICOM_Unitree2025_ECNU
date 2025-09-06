#!/usr/bin/env python3
# encoding: utf-8
import time
import ctypes
import serial
import platform
import serial.tools.list_ports as list_ports
#�ö��Ƽ����߶��ͨ��#

LOBOT_SERVO_FRAME_HEADER         = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = 7
LOBOT_SERVO_MOVE_TIME_WAIT_READ  = 8
LOBOT_SERVO_MOVE_START           = 11
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_ANGLE_OFFSET_READ    = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    = 20
LOBOT_SERVO_ANGLE_LIMIT_READ     = 21
LOBOT_SERVO_VIN_LIMIT_WRITE      = 22
LOBOT_SERVO_VIN_LIMIT_READ       = 23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ  = 25
LOBOT_SERVO_TEMP_READ            = 26
LOBOT_SERVO_VIN_READ             = 27
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ   = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = 32
LOBOT_SERVO_LED_CTRL_WRITE       = 33
LOBOT_SERVO_LED_CTRL_READ        = 34
LOBOT_SERVO_LED_ERROR_WRITE      = 35
LOBOT_SERVO_LED_ERROR_READ       = 36
'''
def pick_port():

    available = {p.device for p in list_ports.comports()}

    for port in ("/dev/ttyUSB2", "/dev/ttyUSB4"):
        if port in available or os.path.exists(port):
            return port

    
    raise FileNotFoundError("0123,0134")
'''    
if platform.system() == "Windows":
        serial_port = "COM4"  # Windows�µĴ���
elif platform.system() == "Linux":
    
    serial_port = "/dev/ttyUSB1"  # Linux�µĴ���
    #serial_port = pick_port()
else:
    raise EnvironmentError("Unsupported platform")
serialHandle = serial.Serial(serial_port, 115200)  # ��ʼ�����ڣ� ������Ϊ115200

def checksum(buf):
    # ����У���
    sum = 0x00
    for b in buf:  # ���
        sum += b
    sum = sum - 0x55 - 0x55  # ȥ�����ͷ������ 0x55
    sum = ~sum  # ȡ��
    return sum & 0xff
    


def serial_serro_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    
    buf = bytearray(b'\x55\x55')  # ֡ͷ
    buf.append(id)
    # ָ���
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    buf.append(w_cmd)  # ָ��
    # д����
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # ƫ��
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # �ֵ�8λ ��8λ ���뻺��
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # �ֵ�8λ ��8λ ���뻺��
    # У���
    buf.append(checksum(buf))
    # for i in buf:
    #     print('%x' %i)
    serialHandle.write(buf)  # ����

def serial_servo_read_cmd(id=None, r_cmd=None):
   
    buf = bytearray(b'\x55\x55')  # ֡ͷ
    buf.append(id)
    buf.append(3)  # ָ���
    buf.append(r_cmd)  # ָ��
    buf.append(checksum(buf))  # У���
    serialHandle.write(buf)  # ����
    time.sleep(0.00034)

def serial_servo_get_rmsg(cmd):
    
    serialHandle.flushInput()  # ��ս��ջ���
    time.sleep(0.005)  # ������ʱ���ȴ��������
    count = serialHandle.inWaiting()    # ��ȡ���ջ����е��ֽ���
    if count != 0:  # ������յ������ݲ���
        recv_data = serialHandle.read(count)  # ��ȡ���յ�������
        # for i in recv_data:
        #     print('%#x' %ord(i))
        # �Ƿ��Ƕ�idָ��
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()  # ��ս��ջ���
                if dat_len == 4:
                    # print ctypes.c_int8(ord(recv_data[5])).value    # ת�����з�������
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
            else:
                return None
        except BaseException as e:
            print(e)
    else:
        serialHandle.flushInput()  # ��ս��ջ���
        return None
