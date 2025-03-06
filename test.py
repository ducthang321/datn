import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk1

GPIO.setmode(GPIO.BCM)
servo_pins = [17, 18, 27]

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

def detect_object(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    colors = {"red": ([0, 120, 70], [10, 255, 255]),
              "green": ([36, 100, 100], [86, 255, 255]),
              "blue": ([94, 80, 100], [126, 255, 255])}
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

def image_to_world(cx, cy, robot, target_color, frame_width=640, frame_height=480):
    z_heights = {"red": 10, "green": 10, "blue": 10}
    z_height = z_heights.get(target_color, 10)
    x_current, y_current, z_current = robot.forwardKinematics()
    camera_offset_x = -20
    camera_offset_y = 0
    camera_offset_z = -10
    x_camera = x_current + camera_offset_x
    y_camera = y_current + camera_offset_y
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
    x = x_camera + offset_x
    y = y_camera + offset_y
    z = z_height
    return x, y, z

def move_to_position(robots, x, y, z):
    q1, q2, q3 = robots.inverseKinematics(x, y, z)
    q3_min, q3_max = robots.q3CalcLimits(q2=q2)
    q3 = max(q3_min, min(q3, q3_max))
    servo_q1, servo_q2, servo_q3 = robots.map_kinematicsToServoAngles(q1=q1, q2=q2, q3=q3)
    set_servo_angle(servo_pins[0], servo_q1)
    set_servo_angle(servo_pins[1], servo_q2)
    set_servo_angle(servo_pins[2], servo_q3)
    print(f"Moved to: q1={q1}, q2={q2}, q3={q3}")

def scan_and_pick(robots, cap, target_color):
    print(f"Scanning for {target_color}...")
    q1 = -30
    found = False
    while q1 <= 30:
        robots.updateJointAngles(q1, 90, 0)
        servo_q1, _, _ = robots.map_kinematicsToServoAngles()
        set_servo_angle(servo_pins[0], servo_q1)
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image")
            return False
        positions = detect_object(frame)
        if target_color in positions:
            cx, cy = positions[target_color]
            print(f"Detected {target_color} at image ({cx}, {cy})")
            x, y, z = image_to_world(cx, cy, robots, target_color)
            print(f"Converted to world coordinates: ({x:.2f}, {y:.2f}, {z:.2f})")
            move_to_position(robots, x, y, z)
            print(f"Picked up {target_color}")
            found = True
            break
        q1 += 15
        time.sleep(0.5)
    if not found:
        print(f"No {target_color} found in scanning range")
    return found

def main():
    robots = EEZYbotARM_Mk1(0, 90, 0)
    target_color = input("Enter target color (red/green/blue): ").strip().lower()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera")
        GPIO.cleanup()
        return
    try:
        scan_and_pick(robots, cap, target_color)
    finally:
        cap.release()
        GPIO.cleanup()
        print("Program ended")

if __name__ == "__main__":
    main()
