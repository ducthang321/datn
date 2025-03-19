import RPi.GPIO as GPIO
import time

# Cấu hình GPIO
GPIO.setmode(GPIO.BCM)
servo_pin = 17
GPIO.setup(servo_pin, GPIO.OUT)

# Khởi tạo PWM với tần số 50Hz (phù hợp với servo)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

# Hàm chuyển đổi góc sang chu kỳ PWM (duty cycle)
def set_angle(angle):
    duty = (angle / 18) + 2.5  # Công thức chuyển đổi góc sang duty cycle
    pwm.ChangeDutyCycle(duty)
    print(f"Servo quay đến góc {angle}°")
    time.sleep(1)  # Cho servo di chuyển

try:
    while True:
        set_angle(0)   # Quay về góc 0°
        time.sleep(4)  # Chờ 4 giây

        set_angle(90)  # Quay đến góc 90°
        time.sleep(4)  # Chờ 4 giây

        set_angle(180) # Quay đến góc 180°
        time.sleep(4)  # Chờ 4 giây

except KeyboardInterrupt:
    print("Dừng chương trình")
    pwm.stop()
    GPIO.cleanup()
