import cv2
import os
import time

# Camera name
camera_name = 'Misumi_200p'
# camera_name = 'Misumi_400x380p'

# Settings
save_dir = 'calibration_images_' + camera_name

if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Camera settings
camera_index = 2  # Change if you have multiple cameras
capture_interval = 1  # Time in seconds between captures
setting_time = 10  # Time in seconds to adjust camera settings
total_images = 20  # Total number of images to capture

# Open the camera
cap = cv2.VideoCapture(camera_index)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()
    
# Show the camera feed to adjust settings for setting_time seconds
print(f"Adjust camera settings for {setting_time} seconds.")
for i in range(setting_time):
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    cv2.imshow('Camera Feed', frame)
    cv2.waitKey(1000)  # Display the image for 1000 ms
    print(f"Setting time remaining: {setting_time - i - 1} seconds.")

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
