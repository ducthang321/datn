import cv2

# GStreamer pipeline để dùng camera CSI với OpenCV
gst_pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink"

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Không thể mở camera CSI. Kiểm tra lại!")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Không thể nhận khung hình từ camera!")
        break

    cv2.imshow("CSI Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
