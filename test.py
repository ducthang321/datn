from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk1
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import subprocess

# Cấu hình GPIO cho servo (bao gồm q4 cho kẹp)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # Tắt cảnh báo GPIO
servo_pins = [17, 18, 27, 22]  # Pin cho q1, q2, q3, q4 (q4 cho kẹp)
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# Khởi tạo PWM cho servo
pwm_objects = {pin: GPIO.PWM(pin, 50) for pin in servo_pins}
for pwm in pwm_objects.values():
    pwm.start(0)

def set_servo_angle(pin, angle, pwm_objects):
    """Điều khiển servo tới góc xác định."""
    duty = angle / 18 + 2  # Chuyển đổi góc thành duty cycle
    pwm_objects[pin].ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm_objects[pin].ChangeDutyCycle(0)  # Tắt PWM sau khi di chuyển

def detect_object(frame, target_color):
    """Phát hiện vật thể dựa trên màu và trả về tọa độ."""
    color_ranges = {
        "red": (np.array([0, 0, 100], dtype=np.uint8), np.array([50, 50, 255], dtype=np.uint8)),
        "green": (np.array([0, 100, 0], dtype=np.uint8), np.array([50, 255, 50], dtype=np.uint8)),
        "blue": (np.array([100, 0, 0], dtype=np.uint8), np.array([255, 50, 50], dtype=np.uint8))
    }
    
    if target_color not in color_ranges:
        return None
    
    lower, upper = color_ranges[target_color]
    mask = cv2.inRange(frame, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 500:  # Lọc nhiễu nhỏ
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
    return None

def process_frame(frame, target_color):
    """Xử lý frame: phát hiện vật thể và vẽ lên frame."""
    processed_frame = frame.copy()
    position = detect_object(processed_frame, target_color)
    
    if position:
        cx, cy = position
        cv2.circle(processed_frame, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(processed_frame, f"Detected: {target_color}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    return processed_frame, position

def image_to_world(cx, cy, robot, target_color, frame_width=640, frame_height=480):
    """Chuyển đổi tọa độ ảnh sang tọa độ thực tế (mm)."""
    z_heights = {"red": 10, "green": 10, "blue": 10}
    z_height = z_heights.get(target_color, 10)
    
    x_current, y_current, z_current = robot.forwardKinematics()
    camera_offset_x = -20
    camera_offset_y = 0
    camera_offset_z = -10
    
    x_camera = x_current + camera_offset_x
    y_camera = y_current + camera_offset_y
    z_camera = z_current + camera_offset_z
    
    camera_height = z_camera if z_camera > 0 else 1  # Tránh chia cho 0
    fov_horizontal = 54
    fov_vertical = 41
    
    real_width = 2 * camera_height * np.tan(np.radians(fov_horizontal / 2))
    real_height = 2 * camera_height * np.tan(np.radians(fov_vertical / 2))
    mm_per_pixel_x = real_width / frame_width
    mm_per_pixel_y = real_height / frame_height
    
    offset_x = (cx - frame_width / 2) * mm_per_pixel_x
    offset_y = (cy - frame_height / 2) * mm_per_pixel_y
    
    x = x_camera + offset_x
    y = y_camera + offset_y
    z = z_height
    return x, y, z

def move_to_position(robot, x, y, z, q4=None):
    """Di chuyển cánh tay tới vị trí xác định và điều khiển kẹp nếu có q4."""
    try:
        q1, q2, q3 = robot.inverseKinematics(x, y, z)
        servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles(q1=q1, q2=q2, q3=q3)
        
        if not (0 <= servo_q1 <= 180 and 0 <= servo_q2 <= 180 and 0 <= servo_q3 <= 180):
            raise ValueError("Servo angles for q1, q2, q3 out of range (0-180 degrees)")
        
        set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
        set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
        set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
        
        if q4 is not None:
            if not (0 <= q4 <= 180):
                raise ValueError("Servo angle for q4 out of range (0-180 degrees)")
            set_servo_angle(servo_pins[3], q4, pwm_objects)
            print(f"Gripper moved to q4={q4:.2f}")
        
        print(f"Moved to: q1={q1:.2f}, q2={q2:.2f}, q3={q3:.2f} (Servo: {servo_q1:.2f}, {servo_q2:.2f}, {servo_q3:.2f})")
        return True
    except Exception as e:
        print(f"Error moving to position: {e}")
        return False

def scan_and_pick(robot, process, target_color):
    """Quét và nhặt vật thể với camera hiển thị liên tục."""
    print(f"Scanning for {target_color}...")
    
    # Mở kẹp ban đầu (q4 = 0°)
    set_servo_angle(servo_pins[3], 0, pwm_objects)
    print("Gripper opened (q4=0)")
    
    buffer = b""
    q1 = -30  # Bắt đầu từ -30°
    step = 5  # Bước quét
    
    while q1 <= 30:
        # Cập nhật góc robot
        robot.updateJointAngles(q1, 90, -90)
        servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles()
        set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
        set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
        set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
        print(f"Rotated to q1={q1}°")
        
        # Đọc và xử lý frame từ camera
        buffer += process.stdout.read(2048)  # Tăng buffer để đọc nhanh hơn
        a = buffer.find(b'\xff\xd8')  # Đầu frame JPEG
        b = buffer.find(b'\xff\xd9')  # Cuối frame JPEG
        if a != -1 and b != -1 and a < b:
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            if frame is None:
                print("Failed to decode frame")
                continue
            
            # Xử lý frame và hiển thị
            processed_frame, position = process_frame(frame, target_color)
            cv2.imshow("Camera Feed", processed_frame)
            
            # Nếu phát hiện vật thể, nhặt nó
            if position:
                cx, cy = position
                x, y, z = image_to_world(cx, cy, robot, target_color)
                print(f"Found {target_color} at ({x:.2f}, {y:.2f}, {z:.2f})")
                
                if move_to_position(robot, x, y, z, q4=0):
                    set_servo_angle(servo_pins[3], 90, pwm_objects)
                    print(f"Gripper closed (q4=90), picked up {target_color}")
                    
                    robot.updateJointAngles(q1, 90, -90)
                    servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles()
                    set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
                    set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
                    set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
                    print("Arm raised after picking")
                    return True
        
        # Kiểm tra phím 'q' để thoát
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quitting scan...")
            break
        
        # Tăng q1
        q1 += step
    
    print(f"No {target_color} found")
    return False

def main():
    """Chương trình chính."""
    robot = EEZYbotARM_Mk1(0, 90, -90)
    
    # Nhập màu trực tiếp từ người dùng thay vì dùng API ngrok
    target_color = input("Enter target color (red, green, blue): ").lower().strip()
    valid_colors = ["red", "green", "blue"]
    if target_color not in valid_colors:
        print(f"Invalid color. Please choose from {valid_colors}")
        GPIO.cleanup()
        return
    
    # Khởi động luồng camera
    cmd = [
        "libcamera-vid",
        "-t", "0",
        "--width", "640",
        "--height", "480",
        "--framerate", "30",
        "--codec", "mjpeg",
        "-o", "-"
    ]
    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)
    except Exception as e:
        print("Error starting camera:", e)
        GPIO.cleanup()
        return
    
    # Tạo cửa sổ OpenCV
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    
    try:
        scan_and_pick(robot, process, target_color)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        process.terminate()
        for pwm in pwm_objects.values():
            pwm.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("Program ended")

if __name__ == "__main__":
    main()
