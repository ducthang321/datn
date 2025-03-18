import numpy as np
import RPi.GPIO as GPIO
from kinematic import EEZYbotARM_Mk1
import time

# Cấu hình GPIO cho servo
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
servo_pins = [17, 18, 27]  # q1, q2, q3
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# Khởi tạo PWM cho servo
pwm_objects = {pin: GPIO.PWM(pin, 50) for pin in servo_pins}
for pwm in pwm_objects.values():
    pwm.start(0)

def set_servo_angle(pin, angle):
    duty = angle / 18 + 2
    pwm_objects[pin].ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm_objects[pin].ChangeDutyCycle(0)

def test_inverse_kinematics():
    # Khởi tạo robot với góc ban đầu bất kỳ
    robot = EEZYbotARM_Mk1(0, 90, -90)
    time.sleep(3)
    # Tọa độ cần kiểm tra
    x_test, y_test, z_test = 137, 0, 141
    
    # Tính toán góc bằng inverse kinematics
    q1, q2, q3 = robot.inverseKinematics(x_test, y_test, z_test)
    
    print(f"Tọa độ đầu vào: ({x_test}, {y_test}, {z_test}) mm")
    print(f"Góc tính toán: q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f}°")
    
    # Điều khiển servo
    set_servo_angle(servo_pins[0], q1)
    set_servo_angle(servo_pins[1], q2)
    set_servo_angle(servo_pins[2], q3)
    print("Servo đã di chuyển đến góc tính toán.")

if __name__ == "__main__":
    try:
        test_inverse_kinematics()
    except KeyboardInterrupt:
        print("Dừng chương trình")
    finally:
        for pwm in pwm_objects.values():
            pwm.stop()
        GPIO.cleanup()
