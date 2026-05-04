#!/usr/bin/env python3

import cv2
import numpy as np

# Load the saved frame
img = cv2.imread("/data/saved_frame.jpg")
if img is None:
    print("ERROR: Could not load /data/saved_frame.jpg")
    print("Make sure you ran save_frame.py first!")
    exit()

height, width = img.shape[:2]

# Crop to bottom 50% - road region only
crop_top = int(height * 0.5)
cropped = img[crop_top:height, 0:width]
cv2.imwrite("/data/00_cropped.jpg", cropped)
print("Saved: 00_cropped.jpg")

# Convert to greyscale for Canny
gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)


# ── EXPERIMENT 1: Canny Edge Detection ───────────────────────────────────────
print("\n--- Canny Edge Detection Experiments ---")

canny_params = [
    (10,  50,  "low_thresholds"),
    (50,  150, "balanced_thresholds"),
    (100, 200, "high_thresholds"),
]

canny_results = []

for low, high, label in canny_params:
    edges = cv2.Canny(gray, low, high)
    filename = f"/data/canny_{label}_{low}_{high}.jpg"
    cv2.imwrite(filename, edges)
    canny_results.append(edges)
    print(f"Saved: canny_{label}_{low}_{high}.jpg  (low={low}, high={high})")


# ── EXPERIMENT 2: Hough Transform ────────────────────────────────────────────
print("\n--- Hough Transform Experiments ---")

balanced_edges = canny_results[1]

hough_thresholds = [10, 30, 80]

for t in hough_thresholds:
    lines = cv2.HoughLinesP(
        balanced_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=t,
        minLineLength=20,
        maxLineGap=10
    )
    output = np.copy(cropped)
    if lines is not None:
        for line in lines:
            l = line[0]
            cv2.line(output, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 2, cv2.LINE_AA)
    filename = f"/data/hough_threshold_{t}.jpg"
    cv2.imwrite(filename, output)
    count = len(lines) if lines is not None else 0
    print(f"Saved: hough_threshold_{t}.jpg  ({count} lines detected)")


# ── EXPERIMENT 3: HSV vs RGB Colour Space ────────────────────────────────────
print("\n--- HSV vs RGB Colour Space Experiments ---")

# HSV yellow filter
hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
lower_yellow_hsv = np.array([20,  80,  80], dtype=np.uint8)
upper_yellow_hsv = np.array([35, 255, 255], dtype=np.uint8)
mask_hsv = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
yellow_hsv = cv2.bitwise_and(cropped, cropped, mask=mask_hsv)
cv2.imwrite("/data/yellow_hsv.jpg", yellow_hsv)
print("Saved: yellow_hsv.jpg")

# RGB yellow filter
lower_yellow_rgb = np.array([100, 100,  0], dtype=np.uint8)
upper_yellow_rgb = np.array([255, 255, 80], dtype=np.uint8)
mask_rgb = cv2.inRange(cropped, lower_yellow_rgb, upper_yellow_rgb)
yellow_rgb = cv2.bitwise_and(cropped, cropped, mask=mask_rgb)
cv2.imwrite("/data/yellow_rgb.jpg", yellow_rgb)
print("Saved: yellow_rgb.jpg")


# ── EXPERIMENT 4: Lighting Conditions ────────────────────────────────────────
print("\n--- Lighting Condition Experiments ---")

# Darken the image
dark_img = cv2.convertScaleAbs(cropped, alpha=0.4, beta=0)
cv2.imwrite("/data/lighting_dark.jpg", dark_img)
print("Saved: lighting_dark.jpg")

# Brighten the image
bright_img = cv2.convertScaleAbs(cropped, alpha=1.6, beta=30)
cv2.imwrite("/data/lighting_bright.jpg", bright_img)
print("Saved: lighting_bright.jpg")

# Run lane detection on both
def detect_lanes(image, label):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0,   0,  180], dtype=np.uint8)
    upper_white = np.array([180, 60, 255], dtype=np.uint8)
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    lower_yellow = np.array([20,  80,  80], dtype=np.uint8)
    upper_yellow = np.array([35, 255, 255], dtype=np.uint8)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                             threshold=30, minLineLength=20, maxLineGap=10)
    output = np.copy(image)
    if lines is not None:
        for line in lines:
            l = line[0]
            cv2.line(output, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 2)
    filename = f"/data/lighting_{label}_result.jpg"
    cv2.imwrite(filename, output)
    count = len(lines) if lines is not None else 0
    print(f"Saved: lighting_{label}_result.jpg  ({count} lines detected)")

detect_lanes(dark_img,   "dark")
detect_lanes(bright_img, "bright")

print("\nAll experiments done! Check /data/ for all saved screenshots.")