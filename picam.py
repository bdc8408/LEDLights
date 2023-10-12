import cv2
import time

# Open a connection to the webcam (0 is usually the default camera)
cap = cv2.VideoCapture(0)

# Set the frame width and height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Create an array to store captured frames
frames = []

# Capture frames
start_time = time.time()
for _ in range(100):
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break
    frames.append(frame)

end_time = time.time()

# Calculate and print the average frame capture time
average_capture_time = (end_time - start_time) / len(frames)
print(f"Average capture time: {average_capture_time} seconds")

# Release the webcam
cap.release()