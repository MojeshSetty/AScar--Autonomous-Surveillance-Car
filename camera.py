import cv2

cap = cv2.VideoCapture(0)  # 0 for default camera
ret, frame = cap.read()  # Capture frame
if ret:
    cv2.imwrite("captured_image.jpg", frame)  # Save image
cap.release()

