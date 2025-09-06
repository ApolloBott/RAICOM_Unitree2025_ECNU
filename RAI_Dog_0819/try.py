import cv2
import numpy as np
import socket
from simple_pid import PID
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import struct
import sys

SEND_HOST = '192.168.123.161'  # 接受pid的IP地址
SEND_PORT = 32000 #接受pid的PORT地址

# 创建UDP套接字
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 加载预定义的ArUco字典（如：DICT_4X4_50）
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# 创建检测参数
parameters = aruco.DetectorParameters()

# 用于存储已检测到的ID集合
detected_ids = set()

global vx_max, vx_min, yaw_max, yaw_min
vx_max = 1
vx_min = -1
yaw_max = 4
yaw_min = -4


# pid 控制部分
def pid_turnright(dxdy,balloon_area):
    max_speed = 2.0  # 最大前进速度
    min_speed = -2.0  # 最大后退速度
    expected_area = 5000 # 目标气球的面积
    yaw_max = 4
    yaw_min = -4
    pid=PID(-0.2,0.01,0.05,setpoint=0)
    pid.output_limits = (yaw_min, yaw_max)
    yawspeed=pid(dxdy)
    pid = PID(1.0, 0.1, 0.05, setpoint=expected_area)
    pid.output_limits = (min_speed, max_speed)  # 设置输出范围
    current_speed=pid(balloon_area)
    return yawspeed,current_speed
def inverse_perspective_transform(image, src_points, dst_size):
    """
    执行逆透视变换的函数。

    参数:
    - image: 输入的图像。
    - src_points: 源点的列表，包含四个点的坐标，每个点是一个(x, y)元组。
    - dst_size: 变换后图像的尺寸，一个(width, height)元组。

    返回:
    - 变换后的图像。
    """
    # 将源点转换为NumPy数组格式
    src_points = np.array(src_points, dtype='float32')

    # 定义目标点，通常是变换后图像的四个角
    dst_points = np.array([
        [0, 0],
        [dst_size[0] - 1, 0],
        [0, dst_size[1] - 1],
        [dst_size[0] - 1, dst_size[1] - 1]
    ], dtype='float32')

    # 计算变换矩阵
    M = cv2.getPerspectiveTransform(src_points, dst_points)

    # 应用逆透视变换
    warped_image = cv2.warpPerspective(image, M, dst_size)

    return warped_image

    # 形态学操作：腐蚀然后膨胀
def filter_noise(binary_image):
    kernel = np.ones((5, 5), np.uint8)
    erosion = cv2.erode(binary_image, kernel, iterations=1)
    dilation = cv2.dilate(erosion, kernel, iterations=1)
    return dilation

def extract_partial_balloon_mask(filtered_mask, min_area=100, max_area=10000, aspect_ratio_threshold=(0.3, 3.0)):
    """
    提取二值化图像中完整或部分气球的范围。

    参数:
    - filtered_mask: 输入的二值化图像。
    - min_area: 连通区域的最小面积，过滤掉噪声和过小的区域。
    - max_area: 连通区域的最大面积，过滤掉可能过大的区域。
    - aspect_ratio_threshold: 宽高比范围 (min_ratio, max_ratio)，用于筛选符合气球形状的区域。

    返回:
    - final_mask: 仅保留气球区域的二值化图像。
    """
    # 初始化输出掩码
    final_mask = np.zeros_like(filtered_mask)

    # 找到所有连通组件
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(filtered_mask, connectivity=8)

    for label in range(1, num_labels):  # 跳过背景
        area = stats[label, cv2.CC_STAT_AREA]
        width = stats[label, cv2.CC_STAT_WIDTH]
        height = stats[label, cv2.CC_STAT_HEIGHT]
        aspect_ratio = width / height if height > 0 else 0

        # 放宽宽高比条件，降低对完整圆形的要求
        if min_area <= area <= max_area and aspect_ratio_threshold[0] <= aspect_ratio <= aspect_ratio_threshold[1]:
            # 将符合条件的组件加入最终掩码
            final_mask[labels == label] = 255

    # 如果最终掩码为空，可以考虑放宽面积限制重新检测
    if np.sum(final_mask) == 0:
        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area > min_area // 2:  # 放宽面积限制
                final_mask[labels == label] = 255

    return final_mask

def calculate_pid_inputs(binary_image, FOV=150):
    """
    根据二值图像计算PID控制所需的输入变量。
    
    参数:
    - binary_image: 二值化后的气球图像。
    - FOV: 摄像头的水平视场角，单位：度。
    
    返回:
    - datax_avr: 垂直方向的加权平均值。
    - dxdy: 气球相对于图像中心的角度控制量。
    - xpos: 气球在图像中的质心水平位置。
    """
    height, width = binary_image.shape

    # 找到所有连通组件
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
    balloon_area = 0  # 初始化气球面积
    
    if num_labels > 1:  # 存在气球区域
        # 选择面积最大的连通组件（排除背景）
        max_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        component_mask = (labels == max_label)

        # 计算质心（center_x, center_y）
        center_x, center_y = centroids[max_label]
        
        # 获取气球区域的面积
        balloon_area = stats[max_label, cv2.CC_STAT_AREA]

        # 获取气球边界点
        # y_indices, x_indices = np.where(component_mask)

        # 计算垂直方向的加权平均值 datax_avr
        # weights = np.linspace(1, 5, len(y_indices)) # 加权从1到5
        # weights_reversed = weights[::-1]
        # datax_avr = np.average(y_indices, weights=weights_reversed)

        # 计算 dxdy：气球相对于图像中心的水平偏移量（单位：像素）
        image_center_x = width / 2  # 图像中心的 x 坐标
        dxdy_pixels = center_x - image_center_x  # 偏移量，左偏为负，右偏为正

        # 将偏移量转换为角度控制量
        angle_per_pixel = FOV / width  # 每个像素对应的角度
        dxdy = dxdy_pixels * angle_per_pixel  # 将偏移量转换为角度

        # 气球质心的水平位置 xpos
        # ypos = center_y

        return  dxdy, balloon_area

    # 如果未检测到气球，返回默认值
    return 0,5000

def main():
    
    # 设置接收端的UDP参数
    HOST = '127.0.0.1'  # 接收图像数据的IP地址
    PORT = 12345  # 接收图像数据的端口号
    BUFF_SIZE = 65535  # 缓冲区大小
    if len(sys.argv) >= 3:
        HOST = sys.argv[1]
        PORT = int(sys.argv[2])
    else:
        print("Usage: python udp_video_from_cpp.py [host] [port]")
        sys.exit(1)

    # 创建UDP接收套接字
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((HOST, PORT))
    print("Waiting for incoming frames...")
    '''
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video capture.")
        return

    desired_fps = 30
    cap.set(cv2.CAP_PROP_FPS, desired_fps)
    '''
    while True:
        try:
            
            # 接收UDP数据
            data, address = server.recvfrom(BUFF_SIZE)
            if len(data) < 10:  # 简单校验，防止接收空数据
                continue
            data = np.array(bytearray(data)) #格式转换
            print("length of data: ", len(data))
            print(data)

            frame = cv2.imdecode(data, 1) #解码
            print('have received one frame')
            print("image size: ({},{})".format(frame.shape[1], frame.shape[0]))
            img_decode_flipped = cv2.flip(frame, -1)  # 参数 -1 表示水平和垂直方向都反转
            cv2.imshow('Frame', img_decode_flipped)
            
            height, width = frame.shape[:2]
            dim = (width, height)
            center = (width // 2, height // 2)
            rotation_matrix = cv2.getRotationMatrix2D(center, 180, 1.0)
            flipped_frame = cv2.warpAffine(frame, rotation_matrix, (width, height))

            resized_frame = cv2.resize(flipped_frame, dim, interpolation=cv2.INTER_AREA)
            
            '''
            # 定义逆透视变换后图像的尺寸
            width_trans = resized_frame.shape[1]
            height_trans = resized_frame.shape[0]
            dst_size = (width_trans, height_trans)

            top_left = (240, 0)
            top_right = (400, 0)
            bottom_left = (0, height_trans)
            bottom_right = (width_trans, height_trans)

            # 定义逆透视源点
            src_points = [top_left, top_right, bottom_left, bottom_right]
            warped_frame = inverse_perspective_transform(resized_frame, src_points, dst_size)
            '''
            # 将帧转换为HSV颜色空间
            hsv = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)
            lower_light_purple1 = np.array([130, 80, 100])  # 色相130到160，饱和度较低，亮度较高
            upper_light_purple1 = np.array([155, 150, 255])
            lower_light_purple2 = np.array([155, 80, 100])
            upper_light_purple2 = np.array([190, 150, 255])
            # 使用inRange函数找到淡紫色区域
            mask1 = cv2.inRange(hsv, lower_light_purple1, upper_light_purple1)
            mask2 = cv2.inRange(hsv, lower_light_purple2, upper_light_purple2)
            # 合并两个淡紫色的掩码
            mask = cv2.bitwise_or(mask1, mask2)
            # 过滤噪声
            filtered_mask = filter_noise(mask)

            # 提取气球的区域
            final_mask = extract_partial_balloon_mask(filtered_mask)

            datax_avr = 0
            datay_avr = 0
            xpos = 0.1
            ypos = 0.1
            dxdy, balloon_area = calculate_pid_inputs(final_mask)
            print(dxdy,balloon_area)
            currentspeed = 0
            yawspeed = 0
            xspeed = 0.2

            # 根据dxdy调整机器人的速度和转向
            
            yawspeed, currentspeed = pid_turnright(dxdy, balloon_area)
            yawspeed_rounded = round(yawspeed, 2)
            currentspeed_values = []
            xspeed_values = []
            yawspeed_values = []
            currentspeed_values.append(str(currentspeed))
            xspeed_values.append(str(xspeed))
            yawspeed_values.append(str(yawspeed_rounded))
                
            if balloon_area<10000:
                data_to_send = '660#' + '#'.join([str(val) for val in currentspeed_values + xspeed_values + yawspeed_values])
                send_socket.sendto(data_to_send.encode('utf-8'), (SEND_HOST, SEND_PORT))
                print("已发送数据:", data_to_send)

            # 检测ArUco标记\
            '''
            gray = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # 如果检测到标记
            if ids is not None:
                for id in ids.flatten():
                    if id not in detected_ids:
                        detected_ids.add(id)
                        print("Detected ArUco marker ID: ", id)
            '''
            # 显示处理后的帧

            cv2.imshow('Binary Image', final_mask)  # 显示二值化图像
            # 每0.1秒输出一次
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break
        except Exception as e:
            print(f"Error: {e}")

    server.close()
    send_socket.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()