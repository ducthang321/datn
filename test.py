import numpy as np
import RPi.GPIO as GPIO
from kinematic import EEZYbotARM_Mk1
import time

GPIO.setmode(GPIO.BCM)
servo_pins = [17, 18, 27, 22]  # q1, q2, q3, q4
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)
# Kh?i t?o PWM cho các servo (50Hz)
pwm_objects = {pin: GPIO.PWM(pin, 50) for pin in servo_pins}
for pwm in pwm_objects.values():
    pwm.start(0)

def set_servo_angle(pin, angle, pwm_objects):
    """Ði?u khi?n servo t?i góc xác d?nh."""
    duty = angle / 18 + 2  # Chuy?n d?i góc thành duty cycle
    pwm_objects[pin].ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm_objects[pin].ChangeDutyCycle(0)  # T?t PWM sau khi di chuy?n

def test_inverse_kinematics():
    # Kh?i t?o robot v?i góc ban d?u b?t k?
    robot = EEZYbotARM_Mk1(15, 90, -110)
    time.sleep(3)
    # T?a d? c?n ki?m tra
    x_test, y_test, z_test = 137, 0, 141
    
    # Tính toán góc b?ng inverse kinematics
    q1, q2, q3 = robot.inverseKinematics(x_test, y_test, z_test)
    servo_q1, servo_q2, servo_q3 = robot.map_kinematicsToServoAngles(q1=q1, q2=q2, q3=q3)
    
    print(f"T?a d? d?u vào: ({x_test}, {y_test}, {z_test}) mm")
    print(f"Góc tính toán: q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f}°")
    
    # Ði?u khi?n servo
    set_servo_angle(servo_pins[0], servo_q1, pwm_objects)
    set_servo_angle(servo_pins[1], servo_q2, pwm_objects)
    set_servo_angle(servo_pins[2], servo_q3, pwm_objects)
    print("Servo dã di chuy?n d?n góc tính toán.")
    
if __name__ == "__main__":
    try:
        test_inverse_kinematics()
    except KeyboardInterrupt:
        print("D?ng chuong trình")
    finally:
        for pwm in pwm_objects.values():
            pwm.stop()
        GPIO.cleanup()
