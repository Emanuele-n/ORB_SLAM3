import cv2
import numpy as np
import yaml

# Define the chessboard size
chessboard_size = (9, 6)
frame_size = (200, 200)

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Capture images of the calibration pattern
images = [cv2.imread(f'calibration_images/image_{i}.jpg') for i in range(1, 21)]  # Assuming 20 images

for img in images:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame_size, None, None)

print(f"Camera Matrix: \n{camera_matrix}")
print(f"Distortion Coefficients: \n{dist_coeffs}")


# Create YAML file
calibration_data = {
    'File.version': '1.0',
    'Camera.type': 'PinHole',
    'Camera.width': frame_size[0],
    'Camera.height': frame_size[1],
    'Camera1.fx': float(camera_matrix[0, 0]),
    'Camera1.fy': float(camera_matrix[1, 1]),
    'Camera1.cx': float(camera_matrix[0, 2]),
    'Camera1.cy': float(camera_matrix[1, 2]),
    'Camera1.k1': float(dist_coeffs[0, 0]),
    'Camera1.k2': float(dist_coeffs[0, 1]),
    'Camera1.p1': float(dist_coeffs[0, 2]),
    'Camera1.p2': float(dist_coeffs[0, 3]),
    'Camera1.k3': float(dist_coeffs[0, 4]) if len(dist_coeffs) > 4 else 0.0,
    'Camera.fps': 30,
    'Camera.RGB': 1,
    'Camera.bf': 40.0,  # You may need to adjust this based on your setup
    'Camera.ThDepth': 40.0,  # You may need to adjust this based on your setup
    'ORBextractor.nFeatures': 1000,
    'ORBextractor.scaleFactor': 1.2,
    'ORBextractor.nLevels': 8,
    'ORBextractor.iniThFAST': 20,
    'ORBextractor.minThFAST': 7,
    'Viewer.KeyFrameSize': 0.05,
    'Viewer.KeyFrameLineWidth': 1.0,
    'Viewer.GraphLineWidth': 0.9,
    'Viewer.PointSize': 2.0,
    'Viewer.CameraSize': 0.08,
    'Viewer.CameraLineWidth': 3.0,
    'Viewer.ViewpointX': 0.0,
    'Viewer.ViewpointY': -0.7,
    'Viewer.ViewpointZ': -1.8,
    'Viewer.ViewpointF': 500.0
}

yaml_filename = 'calibration.yaml'
with open(yaml_filename, 'w') as yaml_file:
    yaml_file.write("%YAML:1.0\n")
    yaml.dump(calibration_data, yaml_file, default_flow_style=False)

print(f"Calibration data saved to {yaml_filename}")