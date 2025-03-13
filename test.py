from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk1
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import subprocess

# Cấu hình GPIO cho servo (q1, q2, q3 và q4 cho kẹp)
GPIO.setmode(GPIO.BCM)
servo_pins = [17, 18, 27, 22]  # q1, q2, q3, q4
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# Khởi tạo PWM cho các servo (50Hz)
pwm_objects = {pin: GPIO.PWM(pin, 50) for pin in servo_pins}
for pwm in pwm_objects.values():
    pwm.start(0)

def set_servo_angle(pin, angle, pwm_objects):
    """Điều khiển servo tới góc xác định bằng cách chuyển đổi sang duty cycle."""
    duty = angle / 18 + 2
    pwm_objects[pin].ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm_objects[pin].ChangeDutyCycle(0)

def detect_object(frame, target_color):
    """Phát hiện vật thể dựa trên màu (trong không gian màu BGR)."""
    color_ranges = {
        "red": (np.array([0, 0, 100], dtype=np.uint8), np.array([50, 50, 255], dtype=np.uint8)),
        "green": (np.array([0, 100, 0], dtype=np.uint8), np.array([50, 255, 50], dtype=np.uint8)),
        "blue": (np.array([100, 0, 0], dtype=np.uint8), np.array([255, 50, 50], dtype=np.uint8))
    }
    
    if target_color not in color_ranges:
        return {}
    
    lower, upper = color_ranges[target_color]
    mask = cv2.inRange(frame, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    positions = {}
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 500:  # Lọc nhiễu nhỏ
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                positions[target_color] = (cx, cy)
    return positions

def image_to_world(cx, cy, robot, target_color, q1, frame_width=640, frame_height=480):
    """
    Chuyển đổi tọa độ ảnh (cx, cy) sang tọa độ thực tế (mm) của robot.
    
    Các bước thực hiện:
      1. Tính offset từ tâm ảnh (tâm ảnh là (frame_width/2, frame_height/2)).
      2. Chuyển đổi pixel sang mm dựa trên FOV của camera và khoảng cách.
      3. Xác định vị trí camera trong hệ tọa độ robot theo robot.forwardKinematics() 
         và các offset cố định.
      4. Áp dụng phép quay (theo góc q1) cho offset để chuyển từ hệ tọa độ camera sang hệ robot.
      5. Tính tọa độ vật thể trong hệ robot.
    """
    # Chiều cao vật thể (mm) tùy theo màu
    z_heights = {"red": 20, "green": 20, "blue": 20}
    z_height = z_heights.get(target_color, 10)
    
    # Lấy vị trí hiện tại của robot (gốc hệ tọa độ robot)
    x_current, y_current, z_current = robot.forwardKinematics()
    
    # Offset của camera so với robot (khi q1 = 0)
    camera_offset_x = -20  # mm (camera nằm phía sau gốc)
    camera_offset_y = 0    # mm
    camera_offset_z = -10  # mm (camera thấp hơn gốc)
    
    # Tọa độ camera trong hệ robot (khi q1 = 0)
    x_camera = x_current + camera_offset_x
    y_camera = y_current + camera_offset_y
    z_camera = z_current + camera_offset_z
    
    # Ước tính khoảng cách từ camera đến mặt phẳng vật thể
    camera_height = z_camera if z_camera > 0 else 1  # tránh chia cho 0
    
    # FOV của camera (độ)
    fov_horizontal = 54
    fov_vertical = 41
    
    # Kích thước vùng quan sát thực tế tại khoảng cách camera_height
    real_width = 2 * camera_height * np.tan(np.radians(fov_horizontal / 2))
    real_height = 2 * camera_height * np.tan(np.radians(fov_vertical / 2))
    mm_per_pixel_x = real_width / frame_width
    mm_per_pixel_y = real_height / frame_height
    
    # Tọa độ offset (mm) từ tâm ảnh (với gốc tọa độ ảnh là góc trên bên trái)
    offset_x = (cx - frame_width / 2) * mm_per_pixel_x
    offset_y = (cy - frame_height / 2) * mm_per_pixel_y
    
    # Áp dụng phép quay theo góc q1 để chuyển từ hệ camera sang hệ tọa độ robot
    theta = np.radians(q1)
    rotated_offset_x = offset_x * np.cos(theta) - offset_y * np.sin(theta)
    rotated_offset_y = offset_x * np.sin(theta) + offset_y * np.cos(theta)
    
    # Tọa độ vật thể trong hệ tọa độ robot
    x_obj = x_camera + rotated_offset_x
    y_obj = y_camera + rotated_offset_y
    z_obj = z_height
    
    return x_obj, y_obj, z_obj

def move_to_position(robot, x, y, z, q4=None):
    """Di chuyển cánh tay tới vị trí (x, y, z); nếu cung cấp q4 thì điều khiển kẹp."""
    try:
        q1, q2, q3 = robot.inverseKinematics(x, y, z)
        servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles(q1=q1, q2=q2, q3=q3)
        
        # Kiểm tra giới hạn góc cho các servo
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
        print("Error moving to position:", e)
        return False

def scan_and_pick(robot, process, target_color):
    """
    Quét môi trường bằng cách xoay q1 từ -30° đến 30°.
    Mỗi lần xoay, delay 5 giây để camera có đủ thời gian xử lý ảnh.
    Khi phát hiện vật, sử dụng tọa độ ảnh cùng với giá trị q1 hiện tại
    để tính tọa độ vật thể trong hệ tọa độ robot và di chuyển nhặt vật.
    """
    print(f"Scanning for {target_color}...")
    
    # Mở kẹp ban đầu (q4 = 0°)
    set_servo_angle(servo_pins[3], 0, pwm_objects)
    print("Gripper opened (q4=0)")
    
    buffer = b""
    found_q1 = None  # Lưu lại góc q1 khi phát hiện vật
    
    for q1 in range(-30, 31, 5):
        # Cập nhật góc robot (q1, q2 = 90, q3 = -90) để quét
        robot.updateJointAngles(q1, 90, -90)
        servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles()
        set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
        set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
        set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
        
        print(f"Rotated to q1={q1}° - waiting 5 seconds for image capture...")
        time.sleep(5)  # Delay đủ để camera xử lý
        
        # Đọc dữ liệu từ luồng camera (stream MJPEG)
        buffer += process.stdout.read(1024)
        a = buffer.find(b'\xff\xd8')  # Đầu frame JPEG
        b = buffer.find(b'\xff\xd9')  # Cuối frame JPEG
        if a != -1 and b != -1 and a < b:
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            positions = detect_object(frame, target_color)
            if target_color in positions:
                cx, cy = positions[target_color]
                # Chuyển đổi tọa độ từ ảnh sang tọa độ thực, truyền luôn q1 hiện tại
                x, y, z = image_to_world(cx, cy, robot, target_color, q1)
                print(f"Found {target_color} at (world coords): ({x:.2f}, {y:.2f}, {z:.2f})")
                
                found_q1 = q1  # Lưu lại góc q1 khi phát hiện vật
                
                # Di chuyển đến vị trí vật thể, với kẹp vẫn mở (q4=0)
                if move_to_position(robot, x, y, z, q4=0):
                    # Đóng kẹp để nhặt vật (q4 = 90°)
                    set_servo_angle(servo_pins[3], 90, pwm_objects)
                    print(f"Gripper closed (q4=90), picked up {target_color}")
                    
                    # Nâng cánh tay lên (giữ q1, cập nhật q2, q3 về vị trí an toàn)
                    robot.updateJointAngles(q1, 90, -90)
                    servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles()
                    set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
                    set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
                    set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
                    print("Arm raised after picking")
                    return True
        time.sleep(1)
    
    print(f"No {target_color} found")
    return False

def main():
    """Chương trình chính."""
    # Khởi tạo robot với góc ban đầu (q1=0, q2=90, q3=-90)
    robot = EEZYbotARM_Mk1(0, 90, -90)
    
    # Nhập màu mục tiêu từ bàn phím
    target_color = input("Enter target color (red/green/blue): ").strip().lower()
    if not target_color:
        print("No target color provided")
        GPIO.cleanup()
        return
    
    # Khởi động luồng camera với libcamera-vid
    cmd = [
        "libcamera-vid",
        "-t", "0",            # Chạy liên tục
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
    
    try:
        scan_and_pick(robot, process, target_color)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        process.terminate()
        for pwm in pwm_objects.values():
            pwm.stop()
        GPIO.cleanup()
        print("Program ended")

if __name__ == "__main__":
    main()
