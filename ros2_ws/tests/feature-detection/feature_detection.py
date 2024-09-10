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


# Load the images
dir = "./tests/2/"
img1 = cv2.imread(dir + "cam1.jpg", cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread(dir + "cam3.jpg", cv2.IMREAD_GRAYSCALE)

# Preprocess the images
target_size = img2.shape  # Resize img1 to match the size of img2


# img1_eq = cv2.equalizeHist(img1)
# img2_eq = cv2.equalizeHist(img2)
# eq = cv2.resize(cv2.hconcat((img1, img1_eq)), (0, 0), fx=0.5, fy=0.5)
# cv2.imshow("Equalization", eq)


img1_pre = resize_image(img1, target_size)


# img2_med = cv2.medianBlur(img2, 3)
img2_prep = cv2.GaussianBlur(img2, (5,5), 0)
img2_prep = cv2.bilateralFilter(img2_prep, 11, 0, 0)


# comp = cv2.resize(cv2.hconcat((img2, img2_eq, img2_prep)), (0, 0), fx=0.5, fy=0.5)
# cv2.imshow("comparison", comp)

keypoints1, keypoints2, matches, img_matches = detect_and_match_features(
    img1_pre, img2_prep
)

# img_matches = cv2.resize(img_matches, (0, 0), fx=0.5, fy=0.5)
# Show the matches
cv2.imshow("Matches", img_matches)
cv2.waitKey(0)
cv2.destroyAllWindows()
