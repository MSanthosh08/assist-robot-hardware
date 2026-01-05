import cv2
import numpy as np

# Left camera pipeline (CAM0)
pipeline_left = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1, format=NV12 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

# Right camera pipeline (CAM1)
pipeline_right = (
    "nvarguscamerasrc sensor-id=1 ! "
    "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1, format=NV12 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

cap_left = cv2.VideoCapture(pipeline_left, cv2.CAP_GSTREAMER)
cap_right = cv2.VideoCapture(pipeline_right, cv2.CAP_GSTREAMER)

if not cap_left.isOpened() or not cap_right.isOpened():
    print("Could not open one or both cameras!")
    exit()

while True:
    retL, frameL = cap_left.read()
    retR, frameR = cap_right.read()

    if not retL or not retR:
        print("Frame read failed!")
        break

    # 🔥 Flip both frames vertically
    frameL = cv2.flip(frameL, 0)   # 0 → vertical flip
    frameR = cv2.flip(frameR, 0)

    # Resize both frames
    frameL = cv2.resize(frameL, (320, 240))
    frameR = cv2.resize(frameR, (320, 240))

    # Combine side-by-side
    combined = np.hstack((frameL, frameR))

    cv2.imshow("Stereo Cameras (Flipped)", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
