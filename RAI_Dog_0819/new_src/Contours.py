import cv2
import numpy as np
def ero_and_dil(binary_image):
    kernel = np.ones((31,31),np.uint8)
    erosion = cv2.erode(binary_image, kernel, iterations=1)
    dilation = cv2.dilate(erosion, kernel, iterations=1)
    closed = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
    return closed

def eight_neighbor_growth(img):
    h, w = img.shape
    print(h,w)
    left_grown = np.zeros_like(img)
    right_grown = np.zeros_like(img)
    #visited = np.zeros_like(img)
    offsets = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
    bottom_row = img[h-1, :]
    index = h-1
    left_seed = None
    right_seed = None
    while left_seed is None and right_seed is None:  
        seed_col = np.where(bottom_row > 200)[0]
        print(seed_col)
        if len(seed_col) > 0:
          left_seed = (index, seed_col[0])
          right_seed = (index, seed_col[1])
        else:
            index = index - 1
            bottom_row = img[index, :]
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
            if 0<=ny1<h and 0<=nx1<w:
                if img[ny1,nx1]>200 and left_grown[ny1,nx1]==0:
                    #visited[ny, nx] = 1
                    left_grown[ny1,nx1] = 255
                    left_queue.append((ny1,nx1))
        for dy2, dx2 in offsets:
            ny2, nx2 = y2+dy2, x2+dx2
            if 0<=ny2<h and 0<=nx2<w:
                if img[ny2,nx2]>200 and right_grown[ny2,nx2]==0:
                    #visited[ny, nx] = 1
                    right_grown[ny2,nx2] = 255
                    right_queue.append((ny2,nx2))
    return left_grown,right_grown

image = cv2.imread("E:\\RAI_Dog\\3.jpg",cv2.IMREAD_GRAYSCALE)
h, w = image.shape
blurred = cv2.GaussianBlur(image, (7,7), 0) 
_, binary = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
ero_and_dil_img = ero_and_dil(binary)
edges = cv2.Canny(ero_and_dil_img, threshold1=50, threshold2=150)
#cv2.imshow('Binary Inverted', binary)  
#cv2.imshow('Canny Edges', edges)
left_mask,right_mask = eight_neighbor_growth(edges)
left_contour = np.where(left_mask==255)
right_contour = np.where(right_mask==255)

#print(len(right_contour[0]))
result = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
result[left_mask==255] = (255,0,0)
result[right_mask==255] = (0,255,0)
#特征点：左右轮廓最顶点、最底点、最左点和最右点
left_top = (left_contour[1][0],left_contour[0][0])#(y,x)
left_bottom = (left_contour[1][-1],left_contour[0][-1])
left_leftest = (left_contour[1][np.argmin(left_contour[1])],left_contour[0][np.argmax(left_contour[1])])
left_rightest = (left_contour[1][np.argmax(left_contour[1])],left_contour[0][np.argmax(left_contour[1])])
#print(left_leftest,left_rightest)
right_top = (right_contour[1][0],right_contour[0][0])
right_bottom = (right_contour[1][-1],right_contour[0][-1])
right_leftest = (right_contour[1][np.argmin(right_contour[1])],right_contour[0][np.argmax(right_contour[1])])
right_rightest = (right_contour[1][np.argmax(right_contour[1])],right_contour[0][np.argmax(right_contour[1])])
#print(right_leftest,right_rightest)
#print(left_top,left_bottom,right_top,right_bottom)
#特征值：轮廓下半部分x坐标平均值
left_half_x = left_contour[1][left_contour[0]>=h/2]
left_even_x = np.mean(left_half_x) if left_half_x.size>0 else np.nan
right_half_x = right_contour[1][right_contour[0]>=h/2]
right_even_x = np.mean(right_half_x) if right_half_x.size>0 else np.nan
#特征值：轮廓底部15%部分平均斜率
left_85_x = left_contour[1][left_contour[0]>=h*0.85]
#left_even_agl = (left_85_x[0] - left_85_x[-1]) / h*0.15
right_85_x = right_contour[1][right_contour[0]>=h*0.85]
#right_even_agl = (right_85_x[0] - right_85_x[-1]) / h*0.15
#print(left_even_agl,right_even_agl)
#特征值：每一行左右轮廓之间的距离
width_map = [right_contour[1][i] - left_contour[1][i] for i in range(1,len(left_contour[0]))]
#print(width_map)
#print(left_even_x,right_even_x)

cv2.circle(result,left_top,10,(0,0,255),-1)
cv2.circle(result,left_bottom,10,(0,255,255),-1)
cv2.circle(result,left_leftest,10,(0,0,255),-1)
cv2.circle(result,left_rightest,10,(0,0,255),-1)
cv2.circle(result,right_top,10,(0,0,255),-1)
cv2.circle(result,right_bottom,10,(0,255,255),-1)
cv2.circle(result,right_leftest,10,(0,255,255),-1)
cv2.circle(result,right_rightest,10,(0,255,255),-1)
#cv2.circle(result,(left_contour[1][0],left_contour[0][0]),10,(0,0,255),-1)
#cv2.circle(result,(right_contour[1][0],right_contour[0][0]),10,(0,0,255),-1)
#cv2.circle(result,(left_contour[1][1038],left_contour[0][1038]),10,(0,255,255),-1)
#cv2.circle(result,(right_contour[1][1038],right_contour[0][1038]),10,(0,255,255),-1)

cv2.imshow("CONTOUR IMG", result)
cv2.waitKey(0)
cv2.destroyAllWindows()