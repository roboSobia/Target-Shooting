import cv2
from ultralytics import YOLO

# Load your trained model
model = YOLO('runs/detect/train/weights/best.pt')

# Open the default camera (0)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the current frame
    results = model.predict(source=frame, show=True)

    # Optionally, display the frame using OpenCV (if not already shown by YOLO)
    # cv2.imshow('YOLO Camera', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
