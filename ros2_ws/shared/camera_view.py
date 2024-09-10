import cv2

# Open the video capture with the specific video device
device = '/dev/video0'
cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

# Set the video format to MJPEG
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

if not cap.isOpened():
    print("Error: Could not open video device:", device)
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Display the frame
    cv2.imshow('Live Camera Feed', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
