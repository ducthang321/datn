import cv2

# Mở camera (0 là camera mặc định, có thể thay đổi nếu có nhiều camera)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()  # Đọc frame từ camera
    if not ret:
        break  # Thoát nếu không lấy được frame

    cv2.imshow('Camera', frame)  # Hiển thị hình ảnh

    # Nhấn phím 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng tài nguyên và đóng cửa sổ
cap.release()
cv2.destroyAllWindows()
