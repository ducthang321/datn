from kinematic import EEZYbotARM_Mk1
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import subprocess

# Cấu hình GPIO cho servo
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
servo_pins = [17, 18, 27, 22]  # q1, q2, q3, q4 (kẹp)
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# Khởi tạo PWM cho servo
pwm_objects = {pin: GPIO.PWM(pin, 50) for pin in servo_pins}
for pwm in pwm_objects.values():
    pwm.start(0)

def set_servo_angle(pin, angle, pwm_objects):
    """Điều khiển servo tới góc xác định."""
    duty = angle / 18 + 2
    pwm_objects[pin].ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm_objects[pin].ChangeDutyCycle(0)

def detect_object(frame, target_color):
    """Phát hiện vật thể dựa trên màu và trả về tọa độ tâm."""
    color_ranges = {
        "red": (np.array([0, 120, 70], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
        "green": (np.array([35, 100, 50], dtype=np.uint8), np.array([85, 255, 255], dtype=np.uint8)),
        "blue": (np.array([100, 150, 50], dtype=np.uint8), np.array([140, 255, 255], dtype=np.uint8))
    }
    
    if target_color not in color_ranges:
        print(f"Màu không hợp lệ: {target_color}")
        return None
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower, upper = color_ranges[target_color]
    mask = cv2.inRange(hsv_frame, lower, upper)
    
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 500:
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print(f"Phát hiện {target_color} tại ({cx}, {cy})")
                return (cx, cy)
    print(f"Không phát hiện được {target_color}")
    return None

def process_frame(frame, target_color):
    """Xử lý khung hình: phát hiện vật thể và vẽ điểm."""
    processed_frame = frame.copy()
    position = detect_object(processed_frame, target_color)
    
    if position:
        cx, cy = position
        cv2.circle(processed_frame, (cx, cy), 10, (0, 255, 0), -1)  # Vẽ điểm xanh lá
        cv2.putText(processed_frame, f"Detected: {target_color}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        print(f"Đã vẽ điểm tại ({cx}, {cy})")
    
    return processed_frame, position

def image_to_world(cx, cy, robot, target_color, frame_width=1296, frame_height=972):
    """Chuyển đổi tọa độ ảnh sang tọa độ thực tế (mm) với camera OV5647."""
    z_heights = {"red": 10, "green": 10, "blue": 10}  # Độ cao giả định của vật thể
    z_height = z_heights.get(target_color, 10)
    
    # Vị trí camera trong hệ tọa độ robot (mm) - Có thể hiệu chỉnh
    camera_position = np.array([80, 0, 141])  # [x, y, z]
    
    # Thông số camera OV5647
    focal_length = 3.29  # Tiêu cự (mm)
    fov_horizontal = 72.4  # Góc nhìn ngang (độ)
    aspect_ratio = 4 / 3  # Tỷ lệ khung hình 1296x972
    fov_vertical = 2 * np.degrees(np.arctan(np.tan(np.radians(fov_horizontal / 2)) / aspect_ratio))
    
    # Tính góc từ tâm ảnh
    pixel_center_x = frame_width / 2
    pixel_center_y = frame_height / 2
    angle_x = np.radians((cx - pixel_center_x) * (fov_horizontal / frame_width))
    angle_y = np.radians((cy - pixel_center_y) * (fov_vertical / frame_height))
    
    # Tính tọa độ trong hệ camera (giả sử vật thể ở độ cao z_height dưới camera)
    camera_height = camera_position[2] - z_height  # Khoảng cách từ camera đến vật thể
    if camera_height <= 0:
        camera_height = 1  # Tránh chia cho 0
    
    # Tọa độ trong mặt phẳng camera (dựa trên góc và tiêu cự)
    x_camera = camera_height * np.tan(angle_x)
    y_camera = -camera_height * np.tan(angle_y)  # Dấu âm vì trục y của ảnh ngược với robot
    
    # Tọa độ trong hệ camera (z=0 tại mặt phẳng vật thể)
    P_camera = np.array([x_camera, y_camera, 0])
    
    # Ma trận quay - Có thể hiệu chỉnh (giả sử camera hướng xuống)
    R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # Thử nghiệm: camera hướng xuống
    P_rotated = np.dot(R, P_camera)
    
    # Chuyển sang hệ robot
    P_robot = P_rotated + camera_position
    P_robot[2] = z_height  # Gán độ cao vật thể
    
    # Debug thông tin
    print(f"FOV ngang: {fov_horizontal}°, FOV dọc: {fov_vertical:.1f}°")
    print(f"Camera position: ({camera_position[0]}, {camera_position[1]}, {camera_position[2]}) mm")
    print(f"Angle x: {np.degrees(angle_x):.2f}°, Angle y: {np.degrees(angle_y):.2f}°")
    print(f"Camera height to object: {camera_height:.2f} mm")
    print(f"Tọa độ robot: ({P_robot[0]:.2f}, {P_robot[1]:.2f}, {P_robot[2]:.2f})")
    
    return P_robot[0], P_robot[1], P_robot[2]

def move_to_position(robot, x, y, z, q4=None):
    """Di chuyển robot tới vị trí và điều khiển kẹp."""
    try:
        q1, q2, q3 = robot.inverseKinematics(x, y, z)
        servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles(q1=q1, q2=q2, q3=q3)
        
        if not (0 <= servo_q1 <= 180 and 0 <= servo_q2 <= 180 and 0 <= servo_q3 <= 180):
            raise ValueError("Góc servo q1, q2, q3 ngoài khoảng 0-180")
        
        set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
        set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
        set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
        
        if q4 is not None:
            if not (0 <= q4 <= 180):
                raise ValueError("Góc servo q4 ngoài khoảng 0-180")
            set_servo_angle(servo_pins[3], q4, pwm_objects)
            print(f"Kẹp di chuyển đến q4={q4:.2f}")
        
        print(f"Di chuyển đến: q1={q1:.2f}, q2={q2:.2f}, q3={q3:.2f}")
        return True
    except Exception as e:
        print(f"Lỗi di chuyển: {e}")
        return False

def pick_object(robot, process, target_color):
    """Nhặt vật thể dựa trên màu."""
    print(f"Chuẩn bị nhặt {target_color}...")
    
    set_servo_angle(servo_pins[3], 0, pwm_objects)
    print("Kẹp mở (q4=0)")
    
    buffer = b""
    picked = False
    
    while not picked:
        buffer += process.stdout.read(2048)
        a = buffer.find(b'\xff\xd8')
        b = buffer.find(b'\xff\xd9')
        if a != -1 and b != -1 and a < b:
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            if frame is None:
                print("Không thể giải mã khung hình")
                continue
            
            frame = cv2.resize(frame, (640, 480))
            processed_frame, position = process_frame(frame, target_color)
            cv2.imshow("Camera Feed", processed_frame)
            
            if position:
                cx, cy = position
                cx = cx * (1296 / 640)  # Điều chỉnh theo tỷ lệ
                cy = cy * (972 / 480)
                x, y, z = image_to_world(cx, cy, robot, target_color)
                print(f"Tìm thấy {target_color} tại ({x:.2f}, {y:.2f}, {z:.2f})")
                
                if move_to_position(robot, x, y, z, q4=0):
                    set_servo_angle(servo_pins[3], 90, pwm_objects)
                    print(f"Kẹp đóng (q4=90), nhặt {target_color}")
                    
                    robot.updateJointAngles(0, 90, -90)
                    servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles()
                    set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
                    set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
                    set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
                    print("Cánh tay nâng lên")
                    picked = True
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Thoát chương trình")
            break
    
    if not picked:
        print(f"Không tìm thấy {target_color}")
    return picked

def main():
    """Chương trình chính."""
    # Khởi tạo robot với các góc q1=0°, q2=90°, q3=-90°
    robot = EEZYbotARM_Mk1(0, 90, -90)
    
    # Di chuyển cánh tay về các góc khởi tạo
    print("Khởi động cánh tay về góc mặc định (q1=0°, q2=90°, q3=-90°)...")
    servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles(q1=0, q2=90, q3=-90)
    set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
    set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
    set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
    print(f"Đã đặt góc servo: q1={servo_q1:.2f}°, q2={servo_q2:.2f}°, q3={servo_q3:.2f}°")
    
    # Hiệu chỉnh thủ công vị trí camera (nếu cần)
    print("Nhập vị trí camera (x, y, z) để hiệu chỉnh (nhấn Enter để giữ mặc định [80, 0, 141]):")
    x_cam = input("x: ") or 80
    y_cam = input("y: ") or 0
    z_cam = input("z: ") or 141
    camera_position = np.array([float(x_cam), float(y_cam), float(z_cam)])
    print(f"Vị trí camera được sử dụng: ({camera_position[0]}, {camera_position[1]}, {camera_position[2]}) mm")
    
    target_color = input("Nhập màu (red, green, blue): ").lower().strip()
    valid_colors = ["red", "green", "blue"]
    if target_color not in valid_colors:
        print(f"Màu không hợp lệ. Chọn từ {valid_colors}")
        GPIO.cleanup()
        return
    
    cmd = ["libcamera-vid", "-t", "0", "--width", "1296", "--height", "972", "--framerate", "30", "--codec", "mjpeg", "-o", "-"]
    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)
    except Exception as e:
        print(f"Lỗi khởi động camera: {e}")
        GPIO.cleanup()
        return
    
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    
    try:
        pick_object(robot, process, target_color)
    except KeyboardInterrupt:
        print("Đã dừng bởi người dùng")
    finally:
        process.terminate()
        for pwm in pwm_objects.values():
            pwm.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("Kết thúc chương trình")

if __name__ == "__main__":
    main()
