import cv2
import numpy as np
import subprocess
import threading

class CameraStream:
    def __init__(self, width=640, height=480, fps=30):
        self.cmd = [
            "libcamera-vid",
            "-t", "0",  # Chạy liên tục
            "--width", str(width),
            "--height", str(height),
            "--framerate", str(fps),
            "--codec", "mjpeg",
            "-o", "-"  # Xuất video ra stdout
        ]
        self.process = None
        self.buffer = b""
        self.frame = None
        self.running = False

    def start(self):
        """Bắt đầu luồng camera"""
        self.running = True
        self.process = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, bufsize=10**8)
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        """Luồng đọc dữ liệu camera"""
        while self.running:
            self.buffer += self.process.stdout.read(4096)
            a = self.buffer.find(b'\xff\xd8')  # Bắt đầu JPEG
            b = self.buffer.find(b'\xff\xd9')  # Kết thúc JPEG
            if a != -1 and b != -1:
                jpg = self.buffer[a:b+2]
                self.buffer = self.buffer[b+2:]
                self.frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

    def read(self):
        """Lấy frame mới nhất"""
        return self.frame

    def stop(self):
        """Dừng camera"""
        self.running = False
        self.process.terminate()
        self.thread.join()

def detect_red(frame):
    """Nhận diện màu đỏ trong frame"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Ngưỡng màu đỏ (có 2 vùng trong HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Tạo mask
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Lọc nhiễu
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))

    # Tìm contours (vùng màu đỏ)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # Lọc vùng nhỏ để tránh nhiễu
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Red Object", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame

# Khởi chạy camera
cam = CameraStream(width=640, height=480, fps=30)
cam.start()

# Vòng lặp chính
while True:
    frame = cam.read()
    if frame is not None:
        frame = detect_red(frame)
        cv2.imshow("Red Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.stop()
cv2.destroyAllWindows()
