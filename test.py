import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk1

# Thiết lập GPIO
GPIO.setmode(GPIO.BCM)
servo_pins = [17, 18, 27]  # Pin cho servo q1, q2, q3
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

pwm_servos = {pin: GPIO.PWM(pin, 50) for pin in servo_pins}
for pwm in pwm_servos.values():
    pwm.start(0)

def set_servo_angle(pin, angle):
    duty = angle / 18 + 2
    pwm_servos[pin].ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm_servos[pin].ChangeDutyCycle(0)

def detect_object(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    colors = {
        "red": ([0, 120, 70], [10, 255, 255]),
        "green": ([36, 100, 100], [86, 255, 255]),
        "blue": ([94, 80, 100], [126, 255, 255])
    }
    positions = {}
    for color, (lower, upper) in colors.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                positions[color] = (cx, cy)
    return positions

def image_to_world(cx, cy, robot, q1_current, target_color, frame_width=640, frame_height=480):
    z_heights = {"red": 10, "green": 10, "blue": 10}
    z_height = z_heights.get(target_color, 10)
    x_current, y_current, z_current = robot.forwardKinematics(q1=q1_current, q2=90, q3=0)
    camera_offset_x = -20
    camera_offset_y = 0
    camera_offset_z = -10
    q1_rad = np.radians(q1_current)
    x_camera = x_current + camera_offset_x * np.cos(q1_rad) - camera_offset_y * np.sin(q1_rad)
    y_camera = y_current + camera_offset_x * np.sin(q1_rad) + camera_offset_y * np.cos(q1_rad)
    z_camera = z_current + camera_offset_z
    camera_height = z_camera
    fov_horizontal = 54
    fov_vertical = 41
    real_width = 2 * camera_height * np.tan(np.radians(fov_horizontal / 2))
    real_height = 2 * camera_height * np.tan(np.radians(fov_vertical / 2))
    mm_per_pixel_x = real_width / frame_width
    mm_per_pixel_y = real_height / frame_height
    offset_x = (cx - frame_width / 2) * mm_per_pixel_x
    offset_y = (cy - frame_height / 2) * mm_per_pixel_y
    x = x_camera + offset_x * np.cos(q1_rad) - offset_y * np.sin(q1_rad)
    y = y_camera + offset_x * np.sin(q1_rad) + offset_y * np.cos(q1_rad)
    z = z_height
    return x, y, z

def move_to_position(robot, x, y, z):
    try:
        q1, q2, q3 = robot.inverseKinematics(x, y, z)
        q3_min, q3_max = robot.q3CalcLimits(q2=q2)
        q3 = max(q3_min, min(q3, q3_max))
        servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles(q1=q1, q2=q2, q3=q3)
        set_servo_angle(servo_pins[0], servo_q1)
        set_servo_angle(servo_pins[1], servo_q2)
        set_servo_angle(servo_pins[2], servo_q3)
        print(f"Moved to: q1={q1:.2f}, q2={q2:.2f}, q3={q3:.2f}")
        return True
    except Exception as e:
        print(f"Error in move_to_position: {e}")
        return False

def scan_and_pick(robot, cap, target_color):
    print(f"Scanning for {target_color}...")
    q1 = -30
    step = 15
    q2_initial = 90  # Trong giới hạn 39 đến 120
    q3_initial = 0   # Sẽ được kiểm tra
    found = False
    
    # Kiểm tra q3_initial có hợp lệ với q2_initial không
    q3_min, q3_max = robot.q3CalcLimits(q2=q2_initial)
    if q3_initial < q3_min or q3_initial > q3_max:
        q3_initial = (q3_min + q3_max) / 2  # Lấy giá trị giữa nếu q3 ban đầu không hợp lệ
        print(f"Adjusted q3_initial to {q3_initial:.2f} to fit limits ({q3_min:.2f}, {q3_max:.2f})")
    
    while q1 <= 30:
        robot.updateJointAngles(q1, q2_initial, q3_initial)
        servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles()
        set_servo_angle(servo_pins[0], servo_q1)
        set_servo_angle(servo_pins[1], servo_q2)
        set_servo_angle(servo_pins[2], servo_q3)
        
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image")
            return False
        
        positions = detect_object(frame)
        if target_color in positions:
            cx, cy = positions[target_color]
            print(f"Detected {target_color} at image ({cx}, {cy}) with q1 = {q1}")
            x, y, z = image_to_world(cx, cy, robot, q1, target_color)
            print(f"Converted to world coordinates: ({x:.2f}, {y:.2f}, {z:.2f})")
            if move_to_position(robot, x, y, z):
                print(f"Picked up {target_color}")
                found = True
            break
        
        q1 += step
        time.sleep(0.5)
    
    if not found:
        print(f"No {target_color} found in scanning range")
    return found

def main():
    robot = EEZYbotARM_Mk1(0, 90, 0)
    target_color = input("Enter target color (red/green/blue): ").strip().lower()
    if target_color not in ["red", "green", "blue"]:
        print("Invalid color. Please choose red, green, or blue.")
        return
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera")
        GPIO.cleanup()
        return
    
    try:
        scan_and_pick(robot, cap, target_color)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        cap.release()
        for pwm in pwm_servos.values():
            pwm.stop()
        GPIO.cleanup()
        print("Program ended")

if __name__ == "__main__":
    main()
