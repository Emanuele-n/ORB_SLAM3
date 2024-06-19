import cv2

# Open a connection to the camera
cap = cv2.VideoCapture(2)  # Change the index if you have multiple cameras

# Capture a single frame to determine the resolution
ret, frame = cap.read()
if ret:
    frame_size = (frame.shape[1], frame.shape[0])
    # Get frame rate
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Frame size: {frame_size}")
    print(f"Frame rate: {fps}")
else:
    print("Failed to capture an image")

# Release the camera
cap.release()
