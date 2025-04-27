from kinematic import EEZYbotARM_Mk1
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import subprocess
import threading

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

def set_servo_angle(pin, angle, pwm_objects, hold=False):
    """Điều khiển servo tới góc xác định. Nếu hold=True thì không tắt duty cycle."""
    duty = angle / 18 + 2
    pwm_objects[pin].ChangeDutyCycle(duty)
    time.sleep(0.5)
    if not hold:
        pwm_objects[pin].ChangeDutyCycle(0)

def hold_servo_angle(pin, angle, pwm_obj, duration=3):
    """Giữ duty cycle liên tục để giữ lực kẹp trong thời gian nhất định (dành cho q4)."""
    duty = angle / 18 + 2
    end_time = time.time() + duration
    while time.time() < end_time:
        pwm_obj.ChangeDutyCycle(duty)
        time.sleep(0.1)
    pwm_obj.ChangeDutyCycle(0)

def detect_object(frame, target_color):
    color_ranges = {
        "red": (np.array([0, 70, 30], dtype=np.uint8), np.array([25, 255, 255], dtype=np.uint8)),
        "green": (np.array([20, 50, 20], dtype=np.uint8), np.array([100, 255, 255], dtype=np.uint8)),
        "blue": (np.array([80, 80, 20], dtype=np.uint8), np.array([170, 255, 255], dtype=np.uint8))
    }
    if target_color not in color_ranges:
        print(f"Màu không hợp lệ: {target_color}")
        return None
    frame_normalized = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
    hsv_frame = cv2.cvtColor(frame_normalized, cv2.COLOR_BGR2HSV)
    lower, upper = color_ranges[target_color]
    blurred_frame = cv2.GaussianBlur(hsv_frame, (9, 9), 0)
    mask = cv2.inRange(blurred_frame, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 100:
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print(f"Phát hiện {target_color} tại ({cx}, {cy})")
                return (cx, cy)
    print(f"Không phát hiện được {target_color}")
    return None

def process_frame(frame, target_color):
    processed_frame = frame.copy()
    position = detect_object(processed_frame, target_color)
    if position:
        cx, cy = position
        cv2.circle(processed_frame, (cx, cy), 10, (0, 255, 0), -1)
        cv2.putText(processed_frame, f"Detected: {target_color}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        print(f"Đã vẽ điểm tại ({cx}, {cy})")
    return processed_frame, position

def image_to_world(cx, cy, robot, target_color, q1, frame_width=1296, frame_height=972):
    z_heights = {"red": 35, "green": 30, "blue": 30}
    z_height = z_heights.get(target_color, 35)
    camera_offset = np.array([11, 0, 111])
    q1_rad = np.radians(q1)
    R_q1 = np.array([
        [np.cos(q1_rad), -np.sin(q1_rad), 0],
        [np.sin(q1_rad), np.cos(q1_rad), 0],
        [0, 0, 1]
    ])
    camera_position = np.dot(R_q1, camera_offset)
    focal_length = 3.29
    fov_horizontal = 72.4
    aspect_ratio = 4 / 3
    fov_vertical = 2 * np.degrees(np.arctan(np.tan(np.radians(fov_horizontal / 2)) / aspect_ratio))
    pixel_origin_x = frame_width / 2
    pixel_origin_y = frame_height
    angle_x = np.radians((pixel_origin_y - cy) * (fov_vertical / frame_height))
    angle_y = np.radians((pixel_origin_x - cx) * (fov_horizontal / frame_width))
    camera_height = camera_position[2] - z_height
    if camera_height <= 0:
        camera_height = 1
    x_camera = camera_height * np.tan(angle_x)
    y_camera = camera_height * np.tan(angle_y)
    P_camera = np.array([x_camera, y_camera, 0])
    R_camera = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]])
    R_total = np.dot(R_q1, R_camera)
    P_rotated = np.dot(R_total, P_camera)
    P_robot = P_rotated + camera_position
    P_robot[2] = z_height
    return P_robot[0], P_robot[1], P_robot[2]

def move_to_position(robot, x, y, z, q4=None):
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
            set_servo_angle(servo_pins[3], q4, pwm_objects, hold=True)
        return True, q1
    except Exception as e:
        print(f"Lỗi di chuyển: {e}")
        return False, None

def pick_object(robot, process, target_color):
    print(f"Chuẩn bị nhặt {target_color}...")
    set_servo_angle(servo_pins[3], 0, pwm_objects)  # mở kẹp
    current_q1 = 0.0
    buffer = b""
    picked = False
    while True:
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
            if not picked and position:
                cx, cy = position
                cx = cx * (1296 / 640)
                cy = cy * (972 / 480)
                x, y, z = image_to_world(cx, cy, robot, target_color, current_q1)
                success, new_q1 = move_to_position(robot, x, y, z, q4=0)
                if success:
                    current_q1 = new_q1
                    set_servo_angle(servo_pins[3], 180, pwm_objects, hold=True)
                    threading.Thread(
                        target=hold_servo_angle, 
                        args=(servo_pins[3], 180, pwm_objects[servo_pins[3]], 5)
                    ).start()
                    robot.updateJointAngles(0, 90, -90)
                    servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles()
                    set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
                    set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
                    set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
                    picked = True
                    print(f"Đã nhặt {target_color} và giữ nguyên vật thể trong kẹp")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    return picked

def main():
    robot = EEZYbotARM_Mk1(0, 90, -90)
    servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles(q1=0, q2=90, q3=-90)
    set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
    set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
    set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
    target_color = input("Nhập màu (red, green, blue): ").lower().strip()
    if target_color not in ["red", "green", "blue"]:
        print("Màu không hợp lệ")
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
