import cv2
import numpy as np
import math

#from Receiver import Receiver

class Img_Processor:
    def __init__(self):
        
        self.image = 0
        self.height = 0
        self.width = 0
        self.blurred = 0
        self.binary = 0
        self.ero_and_dil_img = 0
        self.edges = 0
        self.left_contour = 0
        self.right_contour = 0
        self.result = 0
        self.left_top = 0
        self.left_bottom = 0
        self.left_leftest = 0
        self.left_rightest = 0
        self.right_top = 0
        self.right_bottom = 0
        self.right_leftest = 0
        self.right_rightest = 0
        self.left_even_x = 0
        self.right_even_x = 0
        self.left_even_agl = 0
        self.right_even_agl = 0
        self.width_map = 0
        # self.lower_blue = np.array([72, 40, 0])
        self.lower_blue = np.array([85, 3, 95])
        self.upper_blue = np.array([138, 186, 255])
        self.lower_gray = np.array([14, 9, 41])
        self.upper_gray = np.array([42, 124, 255])
        #self.upper_blue = np.array([106, 220, 217])
        self.blue_leftest_pnt = 0
        self.blue_top_pnt = 0
        self.gray_top_pnt = 0
        self.gray_bottom_pnt = 0
        self.get_l_contour = False
        self.get_r_contour = False
        self.finish_visual = True
        self.tag_id = -1 
        self.ids = 0
        self.idss = set()
        
        
    def get_origin(self):

        #self.image = cv2.imread("E:\\RAI_Dog\\3.jpg",cv2.IMREAD_GRAYSCALE)
        #self.height,self.width = self.image.shape
        self.height, self.width = self.image.shape[:2] 
        print(f"hw:{self.height},{self.width}")

    def get_binary(self):

        self.blurred = cv2.GaussianBlur(self.image, (11,11), 0) 
        #_, self.binary = cv2.threshold(self.blurred, 105, 255, cv2.THRESH_BINARY_INV)
        _, self.binary = cv2.threshold(self.blurred, 155, 255, cv2.THRESH_BINARY_INV)
        self.ero_and_dil_img = self.ero_and_dil(self.binary)
        #cv2.imshow("CONTOUR", self.ero_and_dil_img)
        
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

    def get_contours(self):
        
        self.get_l_contour = False
        self.get_r_contour = False
        self.edges = cv2.Canny(self.ero_and_dil_img, threshold1=50, threshold2=150)
        #self.edges[self.height-10:self.height,:] = 0
        #self.edges[self.height-100:self.height:,0:100] = 0
        #cv2.imshow("EDGE IMG", self.edges)
        
        
        #print(self.edges.shape)
        #cv2.imshow("CONTOUR IMG", self.edges)
        left_mask,right_mask = self.eight_neighbor_growth(self.edges)
        self.left_contour = np.where(left_mask==255)
        self.right_contour = np.where(right_mask==255)
        self.result = cv2.cvtColor(self.edges, cv2.COLOR_GRAY2BGR)
        self.result[left_mask==255] = (255,0,0)
        self.result[right_mask==255] = (0,255,0)
        #print(self.left_contour)

    def get_special_pnts(self):

        if self.left_contour is not None and len(self.left_contour) > 0:
            self.left_top = (self.left_contour[1][0],self.left_contour[0][0])#(y,x)
            self.left_bottom = (self.left_contour[1][-1],self.left_contour[0][-1])
            self.left_leftest = (self.left_contour[1][np.argmin(self.left_contour[1])],self.left_contour[0][np.argmin(self.left_contour[1])])
            self.left_rightest = (self.left_contour[1][np.argmax(self.left_contour[1])],self.left_contour[0][np.argmax(self.left_contour[1])])
        else:
            print("未识别到左线")
            #print(left_leftest,left_rightest)
        if self.right_contour is not None and len(self.right_contour) > 0:
            self.right_top = (self.right_contour[1][0],self.right_contour[0][0])
            self.right_bottom = (self.right_contour[1][-1],self.right_contour[0][-1])
            self.right_leftest = (self.right_contour[1][np.argmin(self.right_contour[1])],self.right_contour[0][np.argmin(self.right_contour[1])])
            self.right_rightest = (self.right_contour[1][np.argmax(self.right_contour[1])],self.right_contour[0][np.argmax(self.right_contour[1])])
    
            '''
            cv2.circle(self.result ,self.left_top,10,(0,0,255),-1)
            cv2.circle(self.result ,self.left_bottom,10,(0,255,255),-1)
            cv2.circle(self.result ,self.left_leftest,10,(0,0,255),-1)
            cv2.circle(self.result ,self.left_rightest,10,(0,0,255),-1)
            cv2.circle(self.result ,self.right_top,10,(0,0,255),-1)
            cv2.circle(self.result ,self.right_bottom,10,(0,255,255),-1)
            cv2.circle(self.result ,self.right_leftest,10,(0,255,255),-1)
            cv2.circle(self.result ,self.right_rightest,10,(0,255,255),-1)
            '''
        else:
            print("未识别到右线")

    def get_evens(self):
        
        #if self.left_contour is not None and len(self.left_contour) > 0:
        if self.get_l_contour:
            #left_half_x = self.left_contour[1][self.left_contour[0]]
            left_half_x = self.left_contour[1]
            self.left_even_x = np.mean(left_half_x) if left_half_x.size>0 else np.nan
            print(f"l:{self.left_even_x}")
          #  if 
            #bottom_row1 = self.edges[250,:]
            #bottom_row2 = self.edges[200,:]
            bottom_row1 = self.edges[120,:]
            bottom_row2 = self.edges[70,:]
            if (bottom_row1 > 200).any() and (bottom_row2 > 200).any():
                left_x1 = np.where(bottom_row1 > 200)[0][0]
                left_x2 = np.where(bottom_row2 > 200)[0][0]
                if left_x1 == left_x2:
                    self.left_even_agl = 90
                left_even_tan = 50 / (left_x2 - left_x1)
                left_even_rad = math.atan(left_even_tan)
                self.left_even_agl = math.degrees(left_even_rad)
                print(f"left_agl:{self.left_even_agl}")
            
            '''
            left_85_x = self.left_contour[1][self.left_contour[0]<=self.height/2]
          #  print(left_85_x)
            if left_85_x[0] == left_85_x[-1]:
                self.left_even_agl = 90
            else:
                left_even_tan = len(left_85_x) / (left_85_x[0] - left_85_x[-1])
                print(f"l:{left_85_x[0]},r:{left_85_x[-1]}")
                left_even_rad = math.atan(left_even_tan)
                self.left_even_agl = math.degrees(left_even_rad)
            '''
        else:
            print("未识别到左线")
        #if self.right_contour is not None and len(self.right_contour) > 0:
        
        if self.get_r_contour:
            #right_half_x = self.right_contour[1][self.right_contour[0]]
            right_half_x = self.right_contour[1]
            self.right_even_x = np.mean(right_half_x) if right_half_x.size>0 else np.nan
            print(f"r:{self.right_even_x}")
            bottom_row1 = self.edges[150,:]
            bottom_row2 = self.edges[100,:]
            if (bottom_row1 > 200).any() and (bottom_row2 > 200).any():
                right_x1 = np.where(bottom_row1 > 200)[0][-1]
                right_x2 = np.where(bottom_row2 > 200)[0][-1]
                if right_x1 == right_x2:
                    self.right_even_agl = 90
                right_even_tan = 50 / (right_x2 - right_x1)
                right_even_rad = math.atan(right_even_tan)
                self.right_even_agl = math.degrees(right_even_rad)
            '''
            right_85_x = self.right_contour[1][self.right_contour[0]<=self.height*0.5]
            if right_85_x[0] == right_85_x[-1]:
                self.right_even_agl = 90
            else:    
                right_even_tan = len(right_85_x) / (right_85_x[0] - right_85_x[-1])
                right_even_rad = math.atan(right_even_tan)
                self.right_even_agl = math.degrees(right_even_rad)
                print(self.left_even_agl,self.right_even_agl)
            '''
        else:
            print("未识别到右线")
      #  if self.get_l_contour == True and self.get_r_contour == False:
        #    self.right_even_x = self.left_even_x

    def get_width(self):
        #pass
        if self.left_contour is not None and len(self.left_contour) > 0 and self.right_contour is not None and len(self.right_contour) > 0 and self.get_l_contour and self.get_r_contour:
            self.width_map = (self.right_contour[1][0] - self.left_contour[1][0] + self.right_contour[1][1] - self.left_contour[1][1] + self.right_contour[1][2] - self.left_contour[1][2])/3
        else:
            self.width_map = 0
        print(f"width:{self.width_map}")
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
        #else:
        #    print("未识别到两条线")
        cv2.imshow("CONTOUR IMG", self.result)
        #cv2.waitKey(1)

    def ero_and_dil(self,binary_image):

        kernel = np.ones((31,31),np.uint8)
        erosion = cv2.erode(binary_image, kernel, iterations=1)
        dilation = cv2.dilate(erosion, kernel, iterations=1)
        closed = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
        return closed
    
    def eight_neighbor_growth(self,img):

        #self.height, self.width = img.shape
        #print(h,w)
        left_grown = np.zeros_like(img)
        right_grown = np.zeros_like(img)
        #grown = np.zeros_like(img)
        #visited = np.zeros_like(img)
        offsets = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
        offsets2 = [(-2,-2),(-2,-1),(-2,0),(-2,1),(-2,2),(-1,2),(0,2),(1,2),(2,2),(2,1),(2,0),(2,-1),(2,-2),(1,-2),(0,-2),(-1,-2)]
        #bottom_row = img[self.height-1, :]
        bottom_row = img[self.height - 200, :]
        #index = self.height-1
       # l_index = self.height - 200
       # r_index = self.height - 200
        #bottom_row2 = img[self.height - 250, :]
        #index = self.height-1
        left_seed = None
        right_seed = None
       # row_ = {bottom_row1:self.height - 200, bottom_row2:self.height - 300}
       # row = [bottom_row1,bottom_row2]
       # for bottom_row in row:
            #if (bottom_row == bottom_row1).all():
        l_index = self.height - 200
        r_index = l_index
            #if (bottom_row == bottom_row2).all():
            #    l_index = self.height - 300
            #    r_index = l_index
        while left_seed is None: 
            seed_col = np.where(bottom_row > 200)[0]
            if len(seed_col) > 0:
                left_seed = (l_index, seed_col[0])
               # if len(seed_col) >= 4 and seed_col[0] <= 50 and seed_col[1] <= 80:
                #    left_seed = (l_index, seed_col[2])
            else:
                l_index = l_index - 1
                if l_index == 50:
                    break
                bottom_row = img[l_index, :]
           # if 
            
        left_queue = [left_seed]
        left_queue.append(left_seed)
        left_grown[left_seed] = 255
            #print(left_queue)
          #  print(left_queue != [None,None])
        while (left_queue != [None,None] and left_queue):    
            y1, x1 = left_queue.pop(0)
            for dy1, dx1 in offsets:
                ny1, nx1 = y1+dy1, x1+dx1
                if 0<=ny1<self.height and 0<=nx1<self.width:
                    if img[ny1,nx1]>200 and left_grown[ny1,nx1]==0:
                        #visited[ny, nx] = 1
                        left_grown[ny1,nx1] = 255
                        #print(f"num:{(left_grown == 255).sum()}")
                    if (left_grown == 255).sum() > 3:
                       self.get_l_contour = True
                       break
                       left_queue.append((ny1,nx1))
            if self.get_l_contour == True:
                break
            if not left_queue:
                for dy1, dx1 in offsets2:
                    ny1, nx1 = y1+dy1, x1+dx1
                    if 0<=ny1<self.height and 0<=nx1<self.width:
                        if img[ny1,nx1]>200 and left_grown[ny1,nx1]==0:
                            #visited[ny, nx] = 1
                            left_grown[ny1,nx1] = 255
                            left_queue.append((ny1,nx1))
                               
                
        while right_seed is None: 
            seed_col = np.where(bottom_row > 200)[0]
            if len(seed_col) > 1:
                right_seed = (r_index, seed_col[-1])
            else:
                r_index = r_index - 1
                if r_index == 50:
                    break
                bottom_row = img[r_index, :]
                
                
                
        right_queue = [right_seed]
        right_queue.append(right_seed)
        right_grown[right_seed] = 255
        while (right_queue != [None,None] and right_queue):
            y1, x1 = right_queue.pop(0)
            for dy1, dx1 in offsets:
                ny1, nx1 = y1+dy1, x1+dx1
                if 0<=ny1<self.height and 0<=nx1<self.width:
                    if img[ny1,nx1]>200 and right_grown[ny1,nx1]==0:
                        #visited[ny, nx] = 1
                        right_grown[ny1,nx1] = 255
                    if (right_grown == 255).sum() > 3:
                        self.get_r_contour = True
                        break
                        right_queue.append((ny1,nx1))
            if self.get_r_contour == True:
                break
            if not right_queue:
                for dy1, dx1 in offsets2:
                    ny1, nx1 = y1+dy1, x1+dx1
                    if 0<=ny1<self.height and 0<=nx1<self.width:
                        if img[ny1,nx1]>200 and right_grown[ny1,nx1]==0:
                            #visited[ny, nx] = 1
                            right_grown[ny1,nx1] = 255
                            right_queue.append((ny1,nx1))
                            
        return left_grown,right_grown
        
    def recog_s_e_r(self):

        #self.blue_l_btm_pnt = (0, 0)
        #self.blue_l_top_pnt = (0, 260)
        #self.blue_leftest_pnt_ = 0
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (19, 19))
        blue_img = cv2.dilate(mask, kernel, iterations=2)
        #contours, _ = cv2.findContours(blue_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #if contours:
            # 寻找最大轮廓
        #    max_contour = max(contours, key=cv2.contourArea)
        blue_img_rol = blue_img[200,:]
        blue_img_col = blue_img[:,200]
        
        #self.blue_leftest_pnt = np.where(blue_img_rol > 200)[0][0]
        self.blue_top_pnt = np.where(blue_img_col > 200)[0][-1]
        print(self.blue_leftest_pnt)
        print(f"top:{self.blue_top_pnt}")
        cv2.imshow("BLUE CONTOUR", blue_img)

    def recog_stair(self):
        '''
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_gray, self.upper_gray)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (19, 19))
        gray_img = cv2.dilate(mask, kernel, iterations=2)
        cv2.imshow("GRAY CONTOUR", gray_img)
        '''
        self.get_origin()
        self.get_binary()
        gray_img_rol = self.ero_and_dil_img[:,200]
        #print(gray_img_rol > 200)
        if ((gray_img_rol > 200) != False).any():
            self.gray_top_pnt = np.where(gray_img_rol > 200)[0][0]
            self.gray_bottom_pnt = np.where(gray_img_rol > 200)[0][-1]
            print(self.gray_top_pnt)
            #print(self.gray_bottom_pnt)
   
        
    def recog_tag(self):
        if self.image is None:
            raise FileNotFoundError("无法加载图像")
        """ArUco 标记识别函数"""
        #dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        #detector_params = cv2.aruco.DetectorParameters_create()
        detector_params = cv2.aruco.DetectorParameters()
        #corners, self.ids, rejected = cv2.aruco.detectMarkers(self.image, dictionary, parameters=detector_params)
        detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
        corners, ids, rejected = detector.detectMarkers(self.image)
        if ids is not None:
            detected_ids = [id[0] for id in ids]
            for tag_id in detected_ids:
                if tag_id < 10 and tag_id not in self.idss:
                    self.ids = tag_id
                    self.idss.add(tag_id)
        #if self.ids != None and self.ids not in self.idss and self.ids[0][0] < 10:
        #    self.idss.append(self.ids)
        # 初始化 ArUco 字典（4x4 字典包含50个标记）
       # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        #aruco_params = cv2.aruco.DetectorParameters()
        #detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        #corners, ids, rejected = detector.detectMarkers(self.image)
        #aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        #corners_ori, ids_ori, _ = cv2.aruco.detectMarkers(self.image, aruco_dict)
        # 用于存储检测结果
        #corners_ori, ids_ori, _ = cv2.aruco.detectMarkers(img_processor.image, aruco_dict)
        #corners_trs, ids_trs, _ = cv2.aruco.detectMarkers(trsfmed_color_img, aruco_dict)

        # 处理原始图像检测结果
       ## if ids is not None:
            #self._process_markers(img_processor.image, ids, corners)
        # 处理变换图像检测结果
        #elif ids_trs is not None:
        #    self._process_markers(trsfmed_color_img, ids_trs, corners_trs)
        # 无标记情况
        #else:
         #   self.tag_id = -1
        print(f"ids:{self.ids}")
        print(f"idss:{self.idss}")
        print(f"len:{len(self.idss)}")
      #  print(self.tag_id)
            
        '''
            if len(seed_col) > 1:
                left_seed = (index, seed_col[0])
                right_seed = (index, seed_col[1])
            if len(seed_col) == 1:
                seed = (index, seed_col[0])
            else:
                index = index - 1
                #index = index + 1
                bottom_row = img[index, :]
        if len(seed_col) > 1:
            left_queue = [left_seed]
            right_queue = [right_seed]
            left_queue.append(left_seed)
            right_queue.append(right_seed)
            #visited[seeds] = 1
            left_grown[left_seed] = 255
            right_grown[right_seed] = 255
            while left_queue and right_queue:
                y1, x1 = left_queue.pop(0)
                y2, x2 = right_queue.pop(0)
                for dy1, dx1 in offsets:
                    ny1, nx1 = y1+dy1, x1+dx1
                    if 0<=ny1<self.height and 0<=nx1<self.width:
                        if img[ny1,nx1]>200 and left_grown[ny1,nx1]==0:
                            #visited[ny, nx] = 1
                            left_grown[ny1,nx1] = 255
                            left_queue.append((ny1,nx1))
                if not left_queue or not right_queue:
                    for dy1, dx1 in offsets2:
                        ny1, nx1 = y1+dy1, x1+dx1
                        if 0<=ny1<self.height and 0<=nx1<self.width:
                            if img[ny1,nx1]>200 and left_grown[ny1,nx1]==0:
                                #visited[ny, nx] = 1
                                left_grown[ny1,nx1] = 255
                                left_queue.append((ny1,nx1))
                                break

                for dy2, dx2 in offsets:
                    ny2, nx2 = y2+dy2, x2+dx2
                    if 0<=ny2<self.height and 0<=nx2<self.width:
                        if img[ny2,nx2]>200 and right_grown[ny2,nx2]==0:
                            #visited[ny, nx] = 1
                            right_grown[ny2,nx2] = 255
                            right_queue.append((ny2,nx2))
                if not left_queue or not right_queue:
                    for dy2, dx2 in offsets2:
                        ny2, nx2 = y2+dy2, x2+dx2
                        if 0<=ny2<self.height and 0<=nx2<self.width:
                            if img[ny2,nx2]>200 and left_grown[ny2,nx2]==0:
                                #visited[ny, nx] = 1
                                left_grown[ny2,nx2] = 255
                                left_queue.append((ny2,nx2))
                                break
            return left_grown,right_grown
        elif len(seed_col) == 1:
            queue = [seed]
            queue.append(seed)
            grown[seed] = 255
            while queue:
                y, x = queue.pop(0)
                for dy1, dx1 in offsets:
                    ny1, nx1 = y+dy1, x+dx1
                    if 0<=ny1<self.height and 0<=nx1<self.width:
                        if img[ny1,nx1]>200 and grown[ny1,nx1]==0:
                            #visited[ny, nx] = 1
                            grown[ny1,nx1] = 255
                            queue.append((ny1,nx1))
                if not queue:
                    for dy1, dx1 in offsets2:
                        ny1, nx1 = y+dy1, x+dx1
                        if 0<=ny1<self.height and 0<=nx1<self.width:
                            if img[ny1,nx1]>200 and grown[ny1,nx1]==0:
                                grown[ny1,nx1] = 255
                                queue.append((ny1,nx1))
                                break

            return grown
            '''
    
#img_processor = Img_Processor()

if __name__ == "__main__":

    img_processor = Img_Processor()
    camera_path = "/dev/video0"
    cap = cv2.VideoCapture(camera_path)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 352)   
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 288)  
    
    cap.set(cv2.CAP_PROP_FPS, 30) 
    while True:
        ret, frame = cap.read()
        if not ret:
            print("finish")
            break
        img_processor.image = frame
        #cv2.imshow("Origin Image", frame)
        #if udp_img is not None:
           #cv2.imshow("UDP Image", udp_img)
           #cv2.waitKey(1)
        
       # img_processor.image = udp_img
        #img_processor.recog_s_e_r()
       # hsv = cv2.cvtColor(img_processor.image, cv2.COLOR_BGR2HSV)
       # h, s, v = cv2.split(hsv)
       # h_min, h_max = np.min(h), np.max(h)
       # s_min, s_max = np.min(s), np.max(s)
       # v_min, v_max = np.min(v), np.max(v)
       # print(f"H: [{h_min}, {h_max}]")
       # print(f"S: [{s_min}, {s_max}]")
       # print(f"V: [{v_min}, {v_max}]")
       # cv2.imshow("UDP Image",img_processor.image)      
        img_processor.get_origin()
        img_processor.get_binary()
        img_processor.get_contours()
        img_processor.get_special_pnts()
        img_processor.get_evens()
        img_processor.get_width()
        img_processor.recog_tag()
       # img_processor.recog_stair()
        #print("1")
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
  
    


            
