from easyEEZYbotARM.kinematic_model import EEZYbotARM_Mk1
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import requests

url ='http://xxxxxx.ngrok.io/process'
GPIO.setmode(GPIO.BCM)
servo_pins = [17, 18, 27]
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

def send_request(text):
    data = {"instruction": text}
    response = requests.post(url, json= data)
    if response.status_code == 200:
        response_data = response.json()
        print (f"Output: { response_data['output']}")
        return response_data['output']
    else:
        print("Error in request")
        return None


def set_servo_angle(pin, angle):
    pwm = GPIO.PWM(pin, 50)  # Tần số 50Hz cho servo
    pwm.start(0)
    duty = angle / 18 + 2  # Chuyển góc thành duty cycle
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.stop()

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
# Kiểm tra camera



def image_to_world(cx, cy, robot, target_color, frame_width=640, frame_height=480):
    """
    Chuyển đổi tọa độ ảnh (pixel) sang tọa độ thực tế (mm) với camera thụt vào 2cm và thấp hơn 1cm.

    Input:
    - cx, cy: Tọa độ tâm vật phẩm trong ảnh (pixel).
    - robot: Đối tượng EEZYbotARM_Mk1 để lấy vị trí hiện tại.
    - target_color: Màu của vật phẩm để xác định z_height.
    - frame_width, frame_height: Độ phân giải ảnh (mặc định 640x480).

    Output:
    - x, y, z: Tọa độ thực tế trong hệ tọa độ thế giới (mm).
    """
    # Chiều cao vật phẩm (mm) dựa trên màu - cần đo thực tế
    z_heights = {"red": 10, "green": 10, "blue": 10}
    z_height = z_heights.get(target_color, 10)

    # Lấy vị trí hiện tại của tâm đầu kẹp
    x_current, y_current, z_current = robot.forwardKinematics()

    # Offset của camera so với tâm đầu kẹp
    camera_offset_x = -20  # mm, thụt vào 2cm (âm theo trục x)
    camera_offset_y = 0    # mm, không lệch theo trục y
    camera_offset_z = -10  # mm, thấp hơn 1cm (âm theo trục z)

    # Tính vị trí của camera trong hệ tọa độ thế giới
    x_camera = x_current + camera_offset_x
    y_camera = y_current + camera_offset_y
    z_camera = z_current + camera_offset_z

    # Chiều cao từ camera đến mặt bàn (z=0)
    camera_height = z_camera

    # FOV của Raspberry Pi Camera Rev 1.3
    fov_horizontal = 54  # Độ, ngang
    fov_vertical = 41    # Độ, dọc

    # Tính kích thước thực tế của vùng ảnh tại mặt bàn
    real_width = 2 * camera_height * np.tan(np.radians(fov_horizontal / 2))
    real_height = 2 * camera_height * np.tan(np.radians(fov_vertical / 2))
    mm_per_pixel_x = real_width / frame_width
    mm_per_pixel_y = real_height / frame_height

    # Tính offset từ giữa ảnh đến tâm vật phẩm (từ vị trí camera)
    offset_x = (cx - frame_width / 2) * mm_per_pixel_x
    offset_y = (cy - frame_height / 2) * mm_per_pixel_y

    # Tọa độ thực tế của vật phẩm
    x = x_camera + offset_x
    y = y_camera + offset_y
    z = z_height
    return x, y, z


def move_to_position(x, y, z, robots):
    q1, q2, q3 = robots.inverseKinematics(x, y, z)
    servo_q1, servo_q2, servo_q3 = robots.map_kinematicsToServoAngles(q1=q1, q2=q2, q3=q3)
    set_servo_angle(servo_pins[0], servo_q1)
    set_servo_angle(servo_pins[1], servo_q2)
    set_servo_angle(servo_pins[2], servo_q3)
    print(f"Moved to: q1={q1}, q2={q2}, q3={q3}")

def scan_and_pick(robots, cap, target_color):
    print(f"Scanning for {target_color}...")
    for q1 in range(-30, 31, 5):  # Giữ nguyên phạm vi -30° đến 30°
        robots.updateJointAngles(q1, 90, 0)
        servo_q1, servo_q2, servo_q3 = robots.map_kinematicsToServoAngles()
        set_servo_angle(servo_pins[0], servo_q1)
        set_servo_angle(servo_pins[1], servo_q2)
        set_servo_angle(servo_pins[2], servo_q3)
        
        ret, frame = cap.read()
        if ret:
            positions = detect_object(frame)
            if target_color in positions:
                cx, cy = positions[target_color]
                x, y, z = image_to_world(cx, cy, robots, target_color)
                print(f"Found {target_color} at ({x:.2f}, {y:.2f}, {z:.2f})")
                if move_to_position(robots, x, y, z):
                    robots.updateJointAngles(q1, 90, -30)
                    servo_q1, servo_q2, servo_q3 = robots.map_kinematicsToServoAngles()
                    set_servo_angle(servo_pins[0], servo_q1)
                    set_servo_angle(servo_pins[1], servo_q2)
                    set_servo_angle(servo_pins[2], servo_q3)
                    print(f"Picked up {target_color}")
                    return True
        time.sleep(0.1)
    print(f"No {target_color} found")
    return False
def main():
    robots = EEZYbotARM_Mk1(0, 90, 0)
    text  = input("Enter instruction: ").split().lower()
    if text:
        output = send_request(text)
    target_color = output
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 15)
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
