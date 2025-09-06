import cv2
print(cv2.__version__)
print(hasattr(cv2.aruco, 'ArucoDetector'))  # 新版本应返回 True
print(hasattr(cv2.aruco, 'detectMarkers'))
print(cv2.aruco.ArucoDetector())