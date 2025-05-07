import cv2
import numpy as np

# Depth estimation parameters
FOCAL_LENGTH = 550  # in pixels (adjust based on your camera)
REAL_SQUARE_WIDTH = 0.15  # in meters (e.g., 30 cm, adjust based on your balloon size)

# Define initial color ranges in HSV
color_ranges = {}

def nothing(x):
    pass

def setup_calibration_window():
    """Setup trackbars for HSV calibration."""
    cv2.namedWindow("Color Calibration")
    cv2.createTrackbar("Lower Hue", "Color Calibration", 0, 180, nothing)
    cv2.createTrackbar("Upper Hue", "Color Calibration", 180, 180, nothing)
    cv2.createTrackbar("Lower Saturation", "Color Calibration", 0, 255, nothing)
    cv2.createTrackbar("Upper Saturation", "Color Calibration", 255, 255, nothing)
    cv2.createTrackbar("Lower Value", "Color Calibration", 0, 255, nothing)
    cv2.createTrackbar("Upper Value", "Color Calibration", 255, 255, nothing)

def get_calibrated_hsv():
    """Retrieve HSV range from trackbars."""
    lower_hue = cv2.getTrackbarPos("Lower Hue", "Color Calibration")
    upper_hue = cv2.getTrackbarPos("Upper Hue", "Color Calibration")
    lower_saturation = cv2.getTrackbarPos("Lower Saturation", "Color Calibration")
    upper_saturation = cv2.getTrackbarPos("Upper Saturation", "Color Calibration")
    lower_value = cv2.getTrackbarPos("Lower Value", "Color Calibration")
    upper_value = cv2.getTrackbarPos("Upper Value", "Color Calibration")
    lower_bound = [lower_hue, lower_saturation, lower_value]
    upper_bound = [upper_hue, upper_saturation, upper_value]
    return lower_bound, upper_bound

def calibration_mode(cap):
    """Calibration mode to fit an object inside a square and calculate HSV ranges."""
    setup_calibration_window()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Draw a fixed square in the center of the frame
        height, width, _ = frame.shape
        square_size = 200
        x1, y1 = (width // 2 - square_size // 2, height // 2 - square_size // 2)
        x2, y2 = (width // 2 + square_size // 2, height // 2 + square_size // 2)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Get calibrated HSV ranges
        lower_bound, upper_bound = get_calibrated_hsv()

        # Convert frame to HSV and apply mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(lower_bound, dtype="uint8"), np.array(upper_bound, dtype="uint8"))
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Display the calibration frame
        cv2.imshow("Calibration Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)

        # Save the HSV range when 's' is pressed
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            color_name = input("Enter the color name: ")
            color_ranges[color_name] = (lower_bound, upper_bound)
            print(f"Saved {color_name}: {lower_bound} to {upper_bound}")
        elif key == ord('q'):
            break

    cv2.destroyAllWindows()

def calculate_depth(pixel_width):
    """Calculate depth using the monocular depth equation: Z = (f * W) / w"""
    if pixel_width == 0:
        return float('inf')  # Avoid division by zero
    depth = (FOCAL_LENGTH * REAL_SQUARE_WIDTH) / pixel_width
    return depth

def detect_squares(frame):
    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    detected_squares = []
    
    # Process each color
    for color, (lower, upper) in color_ranges.items():
        # Create mask for the color
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(hsv, lower, upper)
        
        # Optional: Apply morphological operations to reduce noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Approximate the contour to a polygon
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            
            # Check if the contour is a square (4 sides, roughly equal sides)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                
                # Ensure the shape is roughly a square (aspect ratio ~1)
                if 0.8 <= aspect_ratio <= 1.2 and cv2.contourArea(contour) > 500:
                    depth = calculate_depth(w)

                    detected_squares.append({
                        'color': color,
                        'center': (x + w // 2, y + h // 2),
                        'bbox': (x, y, w, h),
                        'depth': depth
                    })
                    # Draw bounding box and label (for visualization)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, f"{color}, {depth:.2f}m", (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    
    return frame, detected_squares

# Main loop
ip_camera_url = 'http://192.168.55.152:8080/video'  # Example for IP Webcam app
cap = cv2.VideoCapture(ip_camera_url)  # Use 0 for webcam, or specify a video file/game window
if not cap.isOpened():
    print("Error: Could not open video capture")
    exit()

# Start calibration mode
print("Starting calibration mode. Press 's' to save a color and 'q' to quit.")
calibration_mode(cap)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect squares
    processed_frame, squares = detect_squares(frame)
    
    # Print detected squares with depth (for targeting logic)
    for square in squares:
        print(f"Detected {square['color']} square at center: {square['center']}, depth: {square['depth']:.2f}m")
    
    # Display the frame
    cv2.imshow('Target Shooter', processed_frame)
    
    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()