import cv2

# Set up the camera source. 0 usually refers to the default camera.
cap = cv2.VideoCapture(2)

# Define the codec and create a VideoWriter object to write the video.
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (400, 380))

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting ...")
            break

        # Write the frame into the file 'output.avi'
        out.write(frame)

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            print("Exiting: 'q' key pressed.")
            break
finally:
    # When everything done, release the capture
    cap.release()
    out.release()
    cv2.destroyAllWindows()
