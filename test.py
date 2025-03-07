import cv2
import numpy as np
import subprocess

def detect_color(frame, lower, upper):
    mask = cv2.inRange(frame, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 500:  # Lọc bỏ nhiễu nhỏ
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append((cx, cy))
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
    return centers

def main():
    color_choice = input("Nhập màu cần nhận diện (red, green, blue): ").strip().lower()
    
    color_ranges = {
        "red": (np.array([0, 0, 100], dtype=np.uint8), np.array([50, 50, 255], dtype=np.uint8)),
        "green": (np.array([0, 100, 0], dtype=np.uint8), np.array([50, 255, 50], dtype=np.uint8)),
        "blue": (np.array([100, 0, 0], dtype=np.uint8), np.array([255, 50, 50], dtype=np.uint8))
    }
    
    if color_choice not in color_ranges:
        print("Màu không hợp lệ! Chỉ chọn red, green hoặc blue.")
        return
    
    lower_color, upper_color = color_ranges[color_choice]
    
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
    
    while True:
        buffer += process.stdout.read(1024)
        a = buffer.find(b'\xff\xd8')  # Tìm điểm bắt đầu của JPEG
        b = buffer.find(b'\xff\xd9')  # Tìm điểm kết thúc của JPEG
        if a != -1 and b != -1:
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            centers = detect_color(frame, lower_color, upper_color)
            
            # Hiển thị thông tin tọa độ
            for center in centers:
                cv2.putText(frame, f"{color_choice.capitalize()}: {center}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    process.terminate()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
