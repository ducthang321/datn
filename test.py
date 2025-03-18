import numpy as np
import RPi.GPIO as GPIO
from kinematic import EEZYbotARM_Mk1
import time

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
    """Điều khiển servo tới góc xác định."""
    duty = angle / 18 + 2  # Chuyển đổi góc thành duty cycle
    pwm_objects[pin].ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm_objects[pin].ChangeDutyCycle(0)  # Tắt PWM sau khi di chuyển

def test_inverse_kinematics():
    # Khởi tạo robot với góc ban đầu bất kỳ
    robot = EEZYbotARM_Mk1(15, 90, -110)
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
