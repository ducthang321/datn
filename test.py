import numpy as np
from kinematic import EEZYbotARM_Mk1

def test_inverse_kinematics():
    # Khởi tạo robot với góc ban đầu bất kỳ
    robot = EEZYbotARM_Mk1(0, 90, -90)
    
    # Tọa độ cần kiểm tra
    x_test, y_test, z_test = 137, 0, 141
    
    # Tính toán góc bằng inverse kinematics
    q1, q2, q3 = robot.inverseKinematics(x_test, y_test, z_test)
    
    print(f"Tọa độ đầu vào: ({x_test}, {y_test}, {z_test}) mm")
    print(f"Góc tính toán: q1={q1:.2f}°, q2={q2:.2f}°, q3={q3:.2f}°")

if __name__ == "__main__":
    test_inverse_kinematics()
