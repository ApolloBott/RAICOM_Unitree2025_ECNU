import cv2
def main():
    camera_path = "/dev/video0"
    
    cap = cv2.VideoCapture(camera_path)

    if not cap.isOpened():
        print("can't open")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 440)   
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)  
    cap.set(cv2.CAP_PROP_FPS, 30)            
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"set: 400x400 | actual: {actual_width:.0f}x{actual_height:.0f}")
    
    while True:
        ret, frame = cap.read()
        height,width = frame.shape[:2] 
        print(height,width)
       
        
        if not ret:
            print("finish")
            break
        
        cv2.imshow('Camera Feed', frame)
        
        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
