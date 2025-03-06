import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Dùng V4L2
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Độ phân giải thấp để chạy mượt hơn
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Không thể đọc dữ liệu từ camera")
        break

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
