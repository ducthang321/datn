import cv2

def open_camera():
    cap = cv2.VideoCapture(0)  # Mở camera
    
    if not cap.isOpened():
        print("Không thể mở camera")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Không thể nhận khung hình từ camera")
            break
        
        cv2.imshow("Camera", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    open_camera()
