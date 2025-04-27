import cv2
import numpy as np
from ultralytics import YOLO
import threading
import serial  # <-- Added for Arduino serial communication
import time
import math
import webcolors
from scipy.spatial import KDTree

# Set prediction confidence threshold (adjust as needed)
CONF_THRESHOLD = 0.7

# Depth estimation parameters
FOCAL_LENGTH = 550  # in pixels (adjust based on your camera)
BALLOON_WIDTH = 0.17  # in meters (e.g., 30 cm, adjust based on your balloon size)

TARGET_COLOR = "green"    # Target color wanted

IMAGE_WIDTH = 640 
IMAGE_HEIGHT = 480

X_CAMERA_FOV = 86  # in degrees
Y_CAMERA_FOV = 53  # in degrees

LASER_OFFSET_CM_X = 7
LASER_OFFSET_CM_Y = 0

depth = 150;

# Function to calculate pan servo angle
def calculate_pan_angle(target_x, image_width, cam_fov):
    center_x = image_width / 2
    fov_rad = math.radians(x_cameraFOV)
    width_at_target = 2 * depth * math.tan(fov_rad / 2)  # cm
    pixels_per_cm_x = image_width / width_at_target
    laser_camera_offset_pixels_x = laser_offset_cm_x * pixels_per_cm_x
    target_x += laser_camera_offset_pixels_x
    angle = 90 - ((target_x - center_x) / image_width) * cam_fov
    angle = round(angle)
    angle = max(0, min(180, angle))
    return angle

# Function to calculate tilt servo angle
def calculate_tilt_angle(target_y, image_height, cam_fov):
    center_y = image_height / 2
    fov_rad_y = math.radians(y_cameraFOV)  # Vertical FOV in radians
    height_at_target = 2 * depth * math.tan(fov_rad_y / 2)  # Height in cm at target distance
    pixels_per_cm_y = image_height / height_at_target  # Pixels per cm in vertical direction
    laser_camera_offset_pixels_y = laser_offset_cm_y * pixels_per_cm_y  # Vertical laser offset in pixels
    target_y += laser_camera_offset_pixels_y
    angle = 90 + ((target_y - center_y) / image_height) * cam_fov
    angle = round(angle)
    angle = max(0, min(180, angle))
    return angle

# def closest_color(requested_color):
#     min_colors = {}
#     for hex_key, name in webcolors.CSS3_HEX_TO_NAMES.items():  # Use .items() to get hex and name
#         r_c, g_c, b_c = webcolors.hex_to_rgb(hex_key)
#         rd = (r_c - requested_color[0]) ** 2
#         gd = (g_c - requested_color[1]) ** 2
#         bd = (b_c - requested_color[2]) ** 2
#         min_colors[rd + gd + bd] = name
#     return min_colors[min(min_colors.keys())]

# def get_color_name(rgb_tuple):
#     # Convert list to tuple if necessary
#     if isinstance(rgb_tuple, list):
#         rgb_tuple = tuple(rgb_tuple)
    
#     # Validate input
#     if not isinstance(rgb_tuple, tuple) or len(rgb_tuple) != 3:
#         raise ValueError("Input must be a tuple or list of 3 integers [R, G, B]")
#     if not all(isinstance(x, int) and 0 <= x <= 255 for x in rgb_tuple):
#         raise ValueError("RGB values must be integers between 0 and 255")
    
#     try:
#         # Try exact match using rgb_to_name
#         return webcolors.rgb_to_name(rgb_tuple, spec='css3')
#     except ValueError:
#         # Fallback to closest color
#         return closest_color(rgb_tuple)

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
model = YOLO(r'TargetDetection\\runs\detect\\train\weights\best.pt')

# --- Arduino Serial Setup ---
# Change 'COM3' to your Arduino's port (check Arduino IDE > Tools > Port)
try:
    arduino = serial.Serial('COM7', 9600, timeout=1)
    time.sleep(2)  # Wait for the connection to establish
    print("Arduino connected successfully.")
except Exception as e:
    print(f"Could not open serial port: {e}")
    arduino = None

class VideoStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret = ret
                self.frame = frame

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else (False, None)

    def release(self):
        self.stopped = True
        self.thread.join()
        self.cap.release()

# Open the IP camera stream using the threaded VideoStream
ip_camera_url = 'http://192.168.1.41:8080/video'  # Example for IP Webcam app
cap = VideoStream(ip_camera_url)

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
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

              # Calculate pixel width of the balloon
            pixel_width = x2 - x1

            # Estimate depth: Z = (f * W) / w
            depth = (FOCAL_LENGTH * BALLOON_WIDTH) / pixel_width if pixel_width > 0 else 0.0

            # Extract the region of interest (ROI) from the frame
            roi = frame[y1:y2, x1:x2]
            # Skip if ROI is empty (could happen due to bounding box issues)
            if roi.size == 0:
                continue

            # Compute the average BGR color in the ROI
            mean_color = [int(round(x)) for x in cv2.mean(roi)[:3]]  # Round floats to integers            # Determine a basic color name from the average color
            print(mean_color)
            color_name = get_color_name(mean_color)

            print(color_name)

            if color_name != TARGET_COLOR:
                continue
            
            pan_angle = calculate_pan_angle(center_x,image_width,x_cameraFOV)
            tilt_angle = calculate_pan_angle(center_y,image_height,y_cameraFOV)

            # --- Send data to Arduino ---
            # Format: color,x,y\n (e.g., red,123,456\n)
            if arduino is not None:
             try:
               data_str = f"{pan_angle},{tilt_angle}\n"
               print(f"Sending to Arduino: {data_str.strip()}")
               arduino.write(data_str.encode('utf-8'))
               time.sleep(0.05)  # Allow Arduino to respond

        # Read response if available
               while arduino.in_waiting:
                line = arduino.readline().decode('utf-8').strip()
                print("Arduino says:", line)
             except Exception as e:
                  print(f"Serial send error: {e}")

            # if arduino is not None:
            #     try:
            #         data_str = f"{color_name},{center_x},{center_y}\n"
            #         arduino.write(data_str.encode('utf-8'))
            #     except Exception as e:
            #         print(f"Serial send error: {e}")

            # Prepare text info including label, color, confidence, and position
            info_text = f"{label} Depth: {depth:.2f}m ({color_name}) {confs[i]:.2f} Pos: ({center_x}, {center_y})"
            balloons_info.append({'box': (x1, y1, x2, y2),
                                   'color': color_name,
                                   'conf': confs[i],
                                   'pos': (center_x, center_y),
                                   'depth': depth})

            # Draw the bounding box on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Put the text label above the bounding box
            cv2.putText(frame, info_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Optionally, display a summary of all detected balloons on the frame
    summary_text = "Detected Balloons: " + ", ".join(
        [f"{info['color']} {info['pos']} ({info['conf']:.2f}, {info.get('depth', 0.0):.2f}m)" for info in balloons_info])
    cv2.putText(frame, summary_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # Show the final frame with overlays
    cv2.imshow('YOLO Balloon Detection', frame)

    # Exit the loop on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if arduino is not None:
    arduino.close()
