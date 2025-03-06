import cv2

# Dùng GStreamer pipeline d? m? camera trên Raspberry Pi OS m?i
pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink"

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Không th? m? camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Không th? d?c d? li?u t? camera")
        break

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
