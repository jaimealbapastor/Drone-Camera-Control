import cv2
import numpy as np

# Camera parameters from the calibration file
camera_matrix = np.array([[757.47769, 0, 307.39466],
                          [0, 758.68072, 263.43947],
                          [0, 0, 1]])

distortion_coefficients = np.array([0.042590, -0.123901, 0.003057, 0.002304, 0.000000])

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break


    undistorted_image = cv2.undistort(frame, camera_matrix, distortion_coefficients)
    cv2.imshow('Undistorted Image', undistorted_image)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
