import socket
import cv2
import numpy as np
from typing import Tuple, Union
HOST_PC = "192.168.123.15"
HOST_PI = "192.168.123.161"
PORT_TO_PI =  52013
PORT_FROM_13 =  52014
PORT_FROM_15 =  12345


class Receiver:
    def __init__(self, receive_mode: int, host: str, port: int):
        # 保持原始成员变量命名
        self.buffer_size_ = 8 * 1024 
        self.receive_mode_ = receive_mode
        self.recver_endpnt_ = (host, port)
        self.length_ = 0
        self.coded_img_ = bytearray()
        self.camera_img_ = None
        self.img_decode_flipped = 0
        self.frame = 0
        self.buffer = b''

        # 模式初始化
        '''
        if self.receive_mode_ == 0:  # READ_UDP
            self.recv_skt_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.recv_skt_.bind(self.recver_endpnt_)
            except socket.error as e:
                print(f"UDP receiver: Fail to open socket: {e}")
        '''
        if self.receive_mode_ == 0:
            self.recv_skt_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_skt_.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size_)
            try:
                self.recv_skt_.bind(self.recver_endpnt_)
            except socket.error as e:
                print(f"UDP receiver: Fail to open socket: {e}")

        elif self.receive_mode_ == 1:  # READ_CAMERA
            self.cap_ = cv2.VideoCapture(cv2.CAP_V4L2)
            if not self.cap_.isOpened():
                print("Camera Failed")
                return

            '''
            # 设置摄像头参数
            frame_size = (480, 270)
            fps = 30
            self.cap_.set(cv2.CAP_PROP_FRAME_WIDTH, frame_size[0])
            self.cap_.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_size[1])
            self.cap_.set(cv2.CAP_PROP_FPS, fps)
            '''
            
        else:
            print("Error ---> Wrong mode of receiver")

    def camera_img(self) -> Union[np.ndarray, None]:
        return self.camera_img_

    def recv_img(self):
        if self.receive_mode_ == 0:  # READ_UDP
            try:
                data, _ = self.recv_skt_.recvfrom(self.buffer_size_)
                #self.buffer += data
                self.coded_img_ = bytearray(data)
                self.length_ = len(self.coded_img_)
                #self.frame = np.array(bytearray(self.buffer))

                #self.length_ = len(self.frame)
            except socket.error as e:
                print(f"UDP接收错误: {e}")

        elif self.receive_mode_ == 1:  # READ_CAMERA
            ret, self.camera_img_ = self.cap_.read()
            if ret:
                self.length_ = self.camera_img_.shape[0] * self.camera_img_.shape[1]
            else:
                print("摄像头读取失败")
                self.length_ = 0

    def mes_length(self) -> int:
        return self.length_

    def udp_img(self) -> Union[np.ndarray, None]:
        if not self.coded_img_ or self.length_ == 0:
            print("空图像数据")
            return None

        try:
            # 转换为OpenCV可解码格式
            nparr = np.frombuffer(self.coded_img_, np.uint8)
            #img = cv2.imdecode(self.coded_img_, 1)
            #img = cv2.imdecode(self.frame, 1)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            self.img_decode_flipped = cv2.flip(img, -1)
            #if self.frame is not None:
            #    self.buffer = b''  
               # cv2.imshow('frames',self.img_decode_flipped) 
            if img is None:
                raise ValueError("解码失败")
            return self.img_decode_flipped
        except Exception as e:
            print(f"图像解码错误: {e}")
            return None
#receiver_15 = Receiver(1, HOST_PC, PORT_TO_PI)
#receiver_13 = Receiver(0, HOST_PC, PORT_FROM_13)

#receiver_pc = Receiver(0, "192.168.123.100", 12345)

# 使用示例
if __name__ == "__main__":
    # UDP模式示例
    receiver_pc = Receiver(0, "192.168.123.15", 12345)
    while True:
        receiver_pc.recv_img()
        udp_img = receiver_pc.udp_img()
        if udp_img is not None:
            cv2.imshow("UDP Image", receiver_pc.img_decode_flipped)
        if cv2.waitKey(1) == 27:
            break    
    receiver_pc.recv_skt_.close()
    cv2.destroyAllWindows()

    '''
    # 摄像头模式示例
    cam_receiver = Receiver(1, "", 0)
    cam_receiver.recv_img()
    cam_img = cam_receiver.camera_img()
    if cam_img is not None:
        cv2.imshow("Camera Image", cam_img)
        cv2.waitKey(0)
    '''
