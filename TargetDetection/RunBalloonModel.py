import cv2
import numpy as np
from ultralytics import YOLO

# Set prediction confidence threshold (adjust as needed)
CONF_THRESHOLD = 0.7

# Helper function to determine a basic color name from BGR values
def get_color_name(bgr):
    # Convert BGR to RGB for easier interpretation
    r, g, b = bgr[2], bgr[1], bgr[0]
    # Simple heuristic to determine the color name
    if r > 200 and g < 100 and b < 100:
        return "red"
    elif r < 100 and g > 200 and b < 100:
        return "green"
    elif r < 100 and g < 100 and b > 200:
        return "blue"
    elif r > 200 and g > 200 and b < 100:
        return "yellow"
    elif r < 50 and g < 50 and b < 50:
        return "black"
    elif r > 200 and g > 200 and b > 200:
        return "white"
    else:
        # Return the RGB tuple if no basic color is matched
        return f"rgb({int(r)}, {int(g)}, {int(b)})"

# Load your trained YOLO model (assumed to detect balloons)
model = YOLO('TargetDetection\\runs\\detect\\train\\weights\\best.pt')

# Open the default camera (0)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the current frame
    results = model.predict(source=frame, show=False)  # show=False so we can control our display

    # List to store detected balloon info for display
    balloons_info = []

    # Process each result (if multiple are returned)
    for result in results:
        # Skip if there are no detected boxes
        if result.boxes is None or len(result.boxes) == 0:
            continue

        # Convert bounding box, class and confidence values to numpy arrays
        boxes = result.boxes.xyxy.cpu().numpy()  # Bounding box coordinates: [x1, y1, x2, y2]
        cls_ids = result.boxes.cls.cpu().numpy()   # Class indices
        confs = result.boxes.conf.cpu().numpy()      # Confidence scores

        # Loop through each detected object
        for i, box in enumerate(boxes):
            # Apply confidence threshold filter
            if confs[i] < CONF_THRESHOLD:
                continue

            # Get label name (ensure your model's class names include 'balloon')
            label = model.names[int(cls_ids[i])]
            # Only process detections labeled as 'balloon'
            if label.lower() != 'balloon':
                continue

            # Convert bounding box coordinates to integers
            x1, y1, x2, y2 = box.astype(int)

            # Calculate the center (x, y) of the bounding box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Extract the region of interest (ROI) from the frame
            roi = frame[y1:y2, x1:x2]
            # Skip if ROI is empty (could happen due to bounding box issues)
            if roi.size == 0:
                continue

            # Compute the average BGR color in the ROI
            mean_color = cv2.mean(roi)[:3]
            # Determine a basic color name from the average color
            color_name = get_color_name(mean_color)

            # Prepare text info including label, color, confidence, and position
            info_text = f"{label} ({color_name}) {confs[i]:.2f} Pos: ({center_x}, {center_y})"
            balloons_info.append({'box': (x1, y1, x2, y2),
                                   'color': color_name,
                                   'conf': confs[i],
                                   'pos': (center_x, center_y)})

            # Draw the bounding box on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Put the text label above the bounding box
            cv2.putText(frame, info_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Optionally, display a summary of all detected balloons on the frame
    summary_text = "Detected Balloons: " + ", ".join(
        [f"{info['color']} {info['pos']} ({info['conf']:.2f})" for info in balloons_info])
    cv2.putText(frame, summary_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # Show the final frame with overlays
    cv2.imshow('YOLO Balloon Detection', frame)

    # Exit the loop on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
