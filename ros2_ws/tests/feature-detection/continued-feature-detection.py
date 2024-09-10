import cv2
import numpy as np
import matplotlib.pyplot as plt


def resize_image(img, target_size):
    return cv2.resize(
        img, (target_size[1], target_size[0]), interpolation=cv2.INTER_LINEAR
    )


def detect_and_match_features(img1, img2):
    # Initialize ORB detector
    orb = cv2.ORB_create(500)

    # Detect keypoints and descriptors with ORB
    keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

    # Create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors
    matches = bf.match(descriptors1, descriptors2)

    # Sort them in the order of their distance
    matches = sorted(matches, key=lambda x: x.distance)

    # Draw matches
    img_matches = cv2.drawMatches(
        img1,
        keypoints1,
        img2,
        keypoints2,
        matches[:50],
        None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )

    return keypoints1, keypoints2, matches, img_matches


cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

if not cap1.isOpened() or not cap2.isOpened():
    print("Error: Could not open video device.")
    exit()

while True:

    # Load the frames
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    if not ret1 or not ret2:
        print("Error: Could not read frame.")
        break

    # Match size
    target_size = frame2.shape  # Resize img1 to match the size of img2
    frame1 = resize_image(frame1, target_size)

    # Grayscale
    frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Gaussian blur
    frame1_prep = cv2.GaussianBlur(frame1, (3, 3), 0)
    frame2_prep = cv2.GaussianBlur(frame2, (3, 3), 0)

    # Detect and match features
    try:
        keypoints1, keypoints2, matches, img_matches = detect_and_match_features(
            frame1_prep, frame2_prep
        )
    except cv2.error:
        continue

    # img_matches = cv2.resize(img_matches, (0, 0), fx=0.5, fy=0.5)
    # Show the matches
    cv2.imshow("Matches", img_matches)
    if cv2.waitKey(100) & 0xFF == ord("q"):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
