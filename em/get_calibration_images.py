import cv2
import os
import time

# Settings
save_dir = 'calibration_images'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Camera settings
camera_index = 2  # Change if you have multiple cameras
capture_interval = 1  # Time in seconds between captures
total_images = 20  # Total number of images to capture

# Open the camera
cap = cv2.VideoCapture(camera_index)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print(f"Starting capture. Capturing {total_images} images every {capture_interval} second(s).")

# Capture images
for i in range(total_images):
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Save the captured frame
    img_filename = os.path.join(save_dir, f'image_{i+1}.jpg')
    cv2.imwrite(img_filename, frame)
    print(f"Captured {img_filename}")

    # Show the frame (optional)
    cv2.imshow('Captured Image', frame)
    cv2.waitKey(500)  # Display the image for 500 ms

    # Wait for the specified interval
    time.sleep(capture_interval)

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()

print("Image capture complete.")
