<div align="center">
  <h1>🎯 AI-Powered Balloon Shooter with YOLO & Pan-Tilt Control 🎈</h1>
</div>

## 🌟 Project Overview

This project implements an automated system to detect and shoot balloons of a specific color using computer vision and robotics. It leverages a YOLO (You Only Look Once) object detection model to identify balloons in real-time from an IP camera feed. A pan-tilt servo mechanism, controlled by an Arduino, aims at the detected target. Once the system locks onto a balloon of the `TARGET_COLOR`, it sends a command to the Arduino to trigger a "shooting" action. The system also remembers previously shot locations to avoid re-engaging the same targets.

## ✨ Key Features

*   **Real-time Balloon Detection:** Uses a custom-trained YOLO model to detect balloons.
*   **Target Color Filtering:** Specifically targets balloons matching a predefined color (e.g., "yellow").
*   **Pan-Tilt Servo Control:** Precisely aims the shooter using a 2-axis pan-tilt mechanism.
*   **Proportional Control (PID-like):** Smooth and responsive aiming using `KP_X` and `KP_Y` gains.
*   **"Already Shot" Prevention:** Remembers angles where shots have been fired to avoid redundant targeting.
*   **Depth Estimation:** Basic depth estimation based on balloon pixel width and known physical width.
*   **Shooter Offset Compensation:** Accounts for the physical offset between the camera and the shooter.
*   **IP Camera Compatibility:** Utilizes an IP camera stream (e.g., DroidCam) for video input.
*   **Threaded Video Stream:** Ensures smoother video processing without blocking the main logic.
*   **Serial Communication:** Interfaces with an Arduino via serial port to control servos and trigger the shooter.
*   **Visual Feedback:** OpenCV window displays the camera feed, detections, crosshairs, target locks, and servo positions.
*   **Target Timeout:** System can terminate if no new unshot balloons are detected for a specified duration.
*   **On-the-fly Tuning:** `KP` (proportional gain) values can be adjusted using keyboard inputs during runtime.

## 🛠️ Tech Stack

*   **Programming Language:** Python
*   **Computer Vision:** OpenCV, Ultralytics YOLO
*   **Robotics Control:** Arduino (C/C++ for firmware)
*   **Communication:** PySerial (Python to Arduino)
*   **Libraries:** NumPy, threading, time, math

## 🔩 Hardware Requirements

1.  **Computer:** To run the Python script (object detection, control logic).
2.  **IP Camera:** A smartphone with an app like DroidCam or any IP camera.
3.  **Arduino:** (e.g., Arduino Uno, Nano) to control servos and the shooter.
4.  **Pan-Tilt Mechanism:**
    *   2 x Servo Motors (e.g., SG90, MG90S)
    *   Pan-tilt bracket
5.  **Shooter Mechanism:**
    *   A laser pointer for aiming verification, or
    *   A simple projectile launcher (e.g., solenoid-based, small air pump) triggered by the Arduino.
6.  **Balloons:** Of various colors, including the `TARGET_COLOR`.
7.  **Power Supply:** For Arduino and servos.
8.  **USB Cable:** For Arduino programming and serial communication.

## 💾 Software & Installation

1.  **Python 3.x:** Install from [python.org](https://www.python.org/).
2.  **Python Libraries:**
    ```bash
    pip install opencv-python ultralytics pyserial numpy
    ```
3.  **YOLO Model:**
    *   A trained YOLO model file (e.g., `best.pt`). Place it in the path specified in the script: `model = YOLO(r'TargetDetection\\runs\detect\\train\weights\best.pt')`. Adjust path as necessary.
4.  **Arduino IDE:** Install from [arduino.cc](https://www.arduino.cc/).
5.  **Arduino Sketch:** You'll need an Arduino sketch that:
    *   Reads serial data in the format "pan_angle,tilt_angle\n" or "SHOOT\n".
    *   Controls the pan and tilt servos to the specified angles.
    *   Triggers the shooter mechanism when "SHOOT" is received.
    *   Sends an "ACK\n" (acknowledgment) back via serial after a shot.
6.  **IP Camera App:** (e.g., DroidCam) installed on your smartphone and PC client (if needed).

## ⚙️ Configuration (Python Script)

Before running, configure these parameters at the top of the Python script:

*   **`CONF_THRESHOLD`**: Minimum confidence for YOLO detections (e.g., `0.7`).
*   **`FOCAL_LENGTH`**: Camera focal length in pixels (requires calibration).
*   **`BALLOON_WIDTH`**: Actual physical width of the balloons in meters (e.g., `0.18`).
*   **`TARGET_COLOR`**: The desired color of balloons to shoot (e.g., `"yellow"`).
*   **`IMAGE_WIDTH`, `IMAGE_HEIGHT`**: Resolution of your camera feed.
*   **`X_CAMERA_FOV`, `Y_CAMERA_FOV`**: Camera's field of view in degrees.
*   **`LASER_OFFSET_CM_X`, `LASER_OFFSET_CM_Y`**: Physical offset of the shooter from the camera center in cm.
*   **`KP_X`, `KP_Y`**: Proportional gains for pan and tilt control.
*   **`CENTER_TOLERANCE`**: Pixel tolerance for considering the target centered.
*   **`MAX_ANGLE_CHANGE`**: Maximum servo angle change per iteration.
*   **`INIT_PAN`, `INIT_TILT`**: Initial/home position for servos (e.g., `90, 90`).
*   **`YOLO Model Path`**: Update path to your `best.pt` file.
*   **`Arduino Serial Port`**: `arduino = serial.Serial('COM8', 9600, timeout=1)` - change `COM8` to your Arduino's port.
*   **`IP Camera URL`**: `ip_camera_url = 'http://192.168.55.215:4747/video'` - update with your camera's URL.
*   **`MOVEMENT_COOLDOWN`**: Minimum time between servo movements.
*   **`NO_BALLOON_TIMEOUT`**: Duration to wait without detecting new unshot balloons before terminating.
*   **`RETURN_TO_CENTER_DELAY`**: **Note:** This variable is used in `time.sleep(RETURN_TO_CENTER_DELAY)` but is not defined in the provided script. You should define this constant (e.g., `RETURN_TO_CENTER_DELAY = 1.0 # seconds`).

## ▶️ How to Run

1.  **Hardware Setup:**
    *   Assemble the pan-tilt mechanism with servos.
    *   Mount the camera and shooter mechanism.
    *   Connect servos and shooter trigger to the Arduino.
    *   Connect the Arduino to your PC via USB.
2.  **Software Setup:**
    *   Install all prerequisite software and libraries.
    *   Upload the Arduino sketch to your Arduino board.
    *   Start your IP camera stream.
3.  **Configure Script:** Modify the Python script parameters as described above.
4.  **Run the Python Script:**
    ```bash
    python your_script_name.py
    ```
5.  An OpenCV window will appear showing the camera feed, detections, and aiming information. The system will start tracking and attempting to shoot target balloons.

## 🔁 Workflow Logic

1.  **Initialization:**
    *   Connect to Arduino and IP camera.
    *   Load the YOLO model.
    *   Set pan-tilt servos to initial positions (`INIT_PAN`, `INIT_TILT`).
2.  **Main Loop:**
    *   Capture a frame from the IP camera.
    *   Perform YOLO inference to detect objects.
    *   **Filter Detections:**
        *   Keep only 'balloon' detections above `CONF_THRESHOLD`.
        *   Estimate depth and average color of each detected balloon.
        *   Filter for balloons matching `TARGET_COLOR`.
        *   Check if the balloon's potential aiming angles have `is_angle_already_shot`.
    *   **Target Selection:** Identify the best unshot target balloon (e.g., highest confidence).
    *   **Aiming & Shooting:**
        *   If a valid target is found:
            *   Calculate the error between the target's center and the frame center (adjusted for shooter offset).
            *   If `is_target_centered` within `CENTER_TOLERANCE`:
                *   Send "SHOOT" command to Arduino.
                *   Wait for "ACK" from Arduino.
                *   Record the `current_pan`, `current_tilt` as a shot angle.
                *   Return servos to `INIT_PAN`, `INIT_TILT`.
            *   Else (target not centered):
                *   Calculate `new_pan`, `new_tilt` angles using proportional control.
                *   Send new angles to Arduino to move servos.
        *   If no unshot target balloons are detected for `NO_BALLOON_TIMEOUT`, the script terminates.
    *   **Display:** Show the processed frame with bounding boxes, crosshairs, status text.
    *   **Keyboard Input:** Listen for 'q' to quit or 'w/a/s/d' to adjust `KP` values.

## 🔧 Tuning & Parameters

*   **`KP_X`, `KP_Y`:** These are crucial for aiming.
    *   Too low: Slow, sluggish aiming.
    *   Too high: Fast, jerky aiming, may overshoot and oscillate.
    *   Tune these using the 'w/a/s/d' keys during runtime for optimal performance.
*   **`CENTER_TOLERANCE`:** Smaller values mean more precise aiming is required before shooting but might take longer to lock.
*   **`MAX_ANGLE_CHANGE`:** Limits how much the servos move in one step, preventing overly rapid movements.
*   **`FOCAL_LENGTH` & `BALLOON_WIDTH`:** Accurate values are essential for reasonable depth estimation. Calibrate `FOCAL_LENGTH` for your specific camera.
*   **`LASER_OFFSET_CM_X`, `LASER_OFFSET_CM_Y`:** Precisely measure these offsets on your physical setup.
*   **`CONF_THRESHOLD`**: Adjust based on your YOLO model's performance. Higher values reduce false positives but might miss some balloons.
*   **`get_color_name()` function:** This is a very basic heuristic. For more robust color detection, consider using HSV color space filtering or more advanced color classification methods.

## ⚠️ Important Notes

*   **Safety First!** If using a projectile launcher, ensure it's aimed safely and operate in a controlled environment. Start with a laser pointer for aiming verification.
*   The `get_color_name()` function is a simple heuristic and may not be accurate for all lighting conditions or subtle color variations.
*   The `RETURN_TO_CENTER_DELAY` constant needs to be defined in the script for the `time.sleep()` call after a shot.
*   Camera calibration (for `FOCAL_LENGTH` and `FOV`) is important for accurate depth and offset calculations.

---

This README should provide a good starting point for understanding and using your project!
