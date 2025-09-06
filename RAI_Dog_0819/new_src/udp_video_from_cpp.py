import cv2
import numpy
import socket
import struct
import sys

HOST = '127.0.0.1'
PORT = 12345

if len(sys.argv) >= 3:
    HOST = sys.argv[1]
    PORT = int(sys.argv[2])
else:
    print("Usage: python udp_video_from_cpp.py [host] [port]")
    sys.exit(1)

buffSize = 65535
buffer = b''
server = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server.bind((HOST,PORT))

print('now waiting for frames...')

while True:
    
    data, address = server.recvfrom(buffSize) #接收编码图像数据
    #if 10 > len(data): #进行简单的校验
    #    continue
    buffer += data
    frame = numpy.array(bytearray(buffer))
    img_decode = cv2.imdecode(frame, 1)
    img_decode_flipped = cv2.flip(img_decode, -1)
    if img_decode_flipped is not None:
                # 成功解码后清空缓冲区
        buffer = b''
                # 显示帧
        cv2.imshow("UDP Video Stream", img_decode_flipped)
   # data = numpy.array(bytearray(data)) #格式转换
    #print("length of data: ", len(data))
    #print(data)

   #img_decode = cv2.imdecode(data, 1) #解码
    #img_decode_flipped = cv2.flip(img_decode, -1)  # 参数 -1 表示水平和垂直方向都反转
    # print('have received one frame')
    #print("image size: ({},{})".format(img_decode_flipped.shape[1], img_decode_flipped.shape[0]))
    #cv2.imshow('frames',img_decode_flipped) #窗口显示
    
    if cv2.waitKey(1) == 27:
        break

server.close()
cv2.destroyAllWindows()
