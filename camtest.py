import cv2
from PIL import Image
from io import BytesIO
import time

# Open a connection to the webcam (0 is usually the default camera)
cap = cv2.VideoCapture(0)

ret, frame = cap.read()

t = []

for i in range(0,100):
    # Read a frame from the webcam
    tick = time.time()
    ret, frame = cap.read()
    tock = time.time()
    t.append(tock-tick)


# Convert the OpenCV frame to a PIL Image
pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

# Display or manipulate the image as needed
print(sum(t)/len(t))