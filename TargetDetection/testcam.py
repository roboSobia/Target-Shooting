import cv2

ip = input("Enter the DroidCam IP address (e.g., 192.168.1.101): ")
port = input("Enter the DroidCam port (e.g., 4747): ")
camera_url = f"http://{ip}:{port}/video"
print(f"Connecting to DroidCam at {camera_url}")
cap = cv2.VideoCapture(camera_url)

if not cap.isOpened():
    print("Failed to connect to DroidCam.")
else:
    print("Connected successfully!")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from the video stream. Exiting...")
            break

        # Display the video stream
        cv2.imshow("DroidCam Video Stream", frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting video stream...")
            break

    cap.release()
    cv2.destroyAllWindows()