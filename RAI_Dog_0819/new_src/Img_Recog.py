#from Img_Processor import img_processor
import cv2
import numpy as np

class Img_Recog:
    def __init__(self):

        self.tag_id = -1  # 等效于 C++ 的 tag_id_
        self.image = cv2.imread("E:\\RAI_Dog\\4.jpg")
        self.blue_l_btm_pnt_ = (0, 0)    # 底部关键点 (初始左上)
        self.blue_l_top_pnt_ = (0, 260)  # 顶部关键点 (初始左下)
        self.blue_leftest_pnt_ = (435, 0)
        self.lower_blue = np.array([72, 40, 0])
        self.upper_blue = np.array([106, 220, 217])

        #self.recog_tag()
        self.recog_s_e_r()

    def recog_tag(self):
        if self.image is None:
            raise FileNotFoundError("无法加载图像")
        """ArUco 标记识别函数"""
        # 初始化 ArUco 字典（4x4 字典包含50个标记）
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, rejected = detector.detectMarkers(self.image)
        #aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        #corners_ori, ids_ori, _ = cv2.aruco.detectMarkers(self.image, aruco_dict)
        # 用于存储检测结果
        #corners_ori, ids_ori, _ = cv2.aruco.detectMarkers(img_processor.image, aruco_dict)
        #corners_trs, ids_trs, _ = cv2.aruco.detectMarkers(trsfmed_color_img, aruco_dict)

        # 处理原始图像检测结果
        if ids is not None:
            self._process_markers(img_processor.image, ids, corners)
        # 处理变换图像检测结果
        #elif ids_trs is not None:
        #    self._process_markers(trsfmed_color_img, ids_trs, corners_trs)
        # 无标记情况
        else:
            self.tag_id = -1
        print(ids)
        print(self.tag_id)

    def recog_s_e_r(self):

        self.blue_l_btm_pnt = (0, 0)
        self.blue_l_top_pnt = (0, 260)
        self.blue_leftest_pnt = (435, 0)
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (19, 19))
        blue_img = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(blue_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # 寻找最大轮廓
            max_contour = max(contours, key=cv2.contourArea)
            
            # 遍历轮廓点更新关键点
            for point in max_contour[:, 0, :]:  # 提取点坐标
                x, y = point[0], point[1]
                
                # 更新底部点 (y坐标最大)
                if y > self.blue_l_btm_pnt[1]:
                    self.blue_l_btm_pnt = (x, y)
                
                # 更新最左点 (x坐标最小)
                if x < self.blue_leftest_pnt[0]:
                    self.blue_leftest_pnt = (x, y)
                
                # 更新顶部点 (y坐标最小)
                if y < self.blue_l_top_pnt[1]:
                    self.blue_l_top_pnt = (x, y)
        # cv2.drawContours(self.image, [max_contour], -1, (255,255,0), 3)
        cv2.circle(self.image, self.blue_l_btm_pnt, 4, (0,0,0), -1)       # 黑色底部点
        cv2.circle(self.image, self.blue_l_top_pnt, 4, (255,255,255), -1) # 白色顶部点
        cv2.circle(self.image, self.blue_leftest_pnt, 4, (0,0,255), -1)   # 红色最左点

        cv2.imshow("BLUE CONTOUR", blue_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def _process_markers(self, img, ids, corners):
        """处理并绘制标记中心点（内部方法）"""
        for i in range(len(ids)):
            self.tag_id = ids[i][0]  # 提取标量ID值
            
            # 计算角点中心坐标
            marker_corners = corners[i][0]
            center = np.mean(marker_corners, axis=0)
            
            # 绘制中心点（绿色实心圆）
            cv2.circle(img, tuple(center.astype(int)), 5, (0, 255, 0), -1)
img_recog = Img_Recog()
# 使用示例
'''
if __name__ == "__main__":
    # 初始化处理器
    img_recog = Img_Recog()
    
    # 假设有以下输入图像（实际需要从文件/摄像头读取）
    #origin_img = cv2.imread("original.jpg")  
    #transformed_img = cv2.imread("transformed.jpg")
    
    # 执行标记识别
    img_recog.recog_tag()
    
    # 显示结果
    #cv2.imshow("Original", origin_img)
    #cv2.imshow("Transformed", transformed_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()'
    '''