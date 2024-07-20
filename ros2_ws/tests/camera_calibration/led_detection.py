import cv2
import numpy as np

# Camera parameters from the calibration file
camera_matrix = np.array(
    [[757.47769, 0, 307.39466], [0, 758.68072, 263.43947], [0, 0, 1]]
)

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

    undistorted = cv2.undistort(frame, camera_matrix, distortion_coefficients)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    blurred = cv2.medianBlur(gray, 5)

    _, binary = cv2.threshold(blurred, 240, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contour_areas = [(cv2.contourArea(contour), contour) for contour in contours]

    # Sort contours by area in descending order and select the top 3
    sorted_contours = sorted(contour_areas, key=lambda x: x[0], reverse=True)[:3]

    led_positions = []
    led_counter = 0
    for area, contour in sorted_contours:
        # Calculate the moments of the contour
        M = cv2.moments(contour)
        if M["m00"] != 0:
            # Calculate the centroid
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            led_positions.append((cx, cy))

            # Draw a circle at the centroid
            if led_counter < 3:
                color = (0, 255, 0)
                size = 3
            elif led_counter < 5:
                color = (0, 165, 255)
                size = 2
            else:
                color = (0, 0, 255)
                size = 1
            cv2.circle(undistorted, (cx, cy), size, color, -1)
            led_counter += 1

    cv2.imshow("Detected LEDs", undistorted)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
