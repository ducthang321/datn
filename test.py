import cv2
import numpy as np
import subprocess

# Chạy libcamera-vid để lấy video
cmd = [
    "libcamera-vid",
    "-t", "0",  # Chạy liên tục
    "--width", "640",
    "--height", "480",
    "--framerate", "30",
    "--codec", "mjpeg",
    "-o", "-"
]
process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)

buffer = b""
color = 'r'  # Mặc định nhận diện màu đỏ

def get_color_ranges(color):
    """Trả về ngưỡng HSV tương ứng với màu cần nhận diện"""
    if color == 'r':  # Màu đỏ
        lower1, upper1 = (0, 120, 70), (10, 255, 255)
        lower2, upper2 = (170, 120, 70), (180, 255, 255)
    elif color == 'g':  # Màu xanh lá
        lower1, upper1 = (35, 100, 50), (85, 255, 255)
        lower2, upper2 = None, None  # Không có vùng màu thứ 2
    elif color == 'b':  # Màu xanh dương
        lower1, upper1 = (100, 120, 70), (140, 255, 255)
        lower2, upper2 = None, None
    return lower1, upper1, lower2, upper2

while True:
    buffer += process.stdout.read(4096)
    a = buffer.find(b'\xff\xd8')  # Tìm JPEG start
    b = buffer.find(b'\xff\xd9')  # Tìm JPEG end

    if a != -1 and b != -1:
        jpg = buffer[a:b+2]
        buffer = buffer[b+2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        # Chuyển ảnh sang HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Lấy ngưỡng màu theo lựa chọn từ bàn phím
        lower1, upper1, lower2, upper2 = get_color_ranges(color)

        # Tạo mask
        mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
        mask = mask1
        if lower2 and upper2:
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask += mask2

        # Tìm contours (vùng có màu)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Tìm vùng màu lớn nhất
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:  # Lọc vùng nhỏ
                x, y, w, h = cv2.boundingRect(largest)
                cx, cy = x + w // 2, y + h // 2  # Tọa độ trung tâm
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"({cx}, {cy})", (cx, cy - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(f"Tọa độ: ({cx}, {cy})")

        # Hiển thị hình ảnh
        cv2.imshow("Color Detection", frame)

    # Nhập màu từ bàn phím
    key = cv2.waitKey(1) & 0xFF
    if key == ord('r'):
        color = 'r'
        print("Nhận diện màu ĐỎ")
    elif key == ord('g'):
        color = 'g'
        print("Nhận diện màu XANH LÁ")
    elif key == ord('b'):
        color = 'b'
        print("Nhận diện màu XANH DƯƠNG")
    elif key == ord('q'):
        break

process.terminate()
cv2.destroyAllWindows()
