import RPi.GPIO as GPIO
import time

# C?u hình GPIO
GPIO.setmode(GPIO.BCM)
servo_pin = 17
GPIO.setup(servo_pin, GPIO.OUT)

# Kh?i t?o PWM v?i t?n s? 50Hz (phù h?p v?i servo)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

# Hàm chuy?n d?i góc sang chu k? PWM (duty cycle)
def set_angle(angle):
    if 0 <= angle <= 180:  # Ki?m tra góc h?p l?
        duty = (angle / 18) + 2.5  # Công th?c chuy?n d?i góc sang duty cycle
        pwm.ChangeDutyCycle(duty)
        print(f"Servo quay d?n góc {angle}°")
        time.sleep(1)  # Cho servo di chuy?n
    else:
        print("Góc không h?p l?! Vui lòng nh?p giá tr? t? 0 d?n 180.")

try:
    while True:
        # Nh?p góc t? bàn phím
        angle_input = input("Nh?p góc (0-180) ho?c 'q' d? thoát: ")
        
        # Ki?m tra n?u ngu?i dùng mu?n thoát
        if angle_input.lower() == 'q':
            print("Thoát chuong trình")
            break
        
        # Chuy?n d?i input thành s? và di?u khi?n servo
        try:
            angle = float(angle_input)
            set_angle(angle)
        except ValueError:
            print("Vui lòng nh?p m?t s? h?p l?!")

except KeyboardInterrupt:
    print("\nD?ng chuong trình b?i ngu?i dùng")

finally:
    pwm.stop()
    GPIO.cleanup()
