import numpy as np
import cv2
import cv2.cv as cv
from domi import calibration, filters, detectors, draw

# Load camera feed
#cap = cv2.VideoCapture(0)
#ret, frame = cap.read()

# Load image
frame = cv2.imread('frame.png')
# Crop only the individual dominoes for calibration
frame = frame[:,0:frame.shape[1] * 0.4]


# Filter
black_white = filters.black_white(frame)

# Compute
pips, edges = detectors.img_to_contour_points(black_white)
corners = detectors.all_contour_points_to_corners(edges)

# Calibrate
calibration.calibrate_with_corners(corners)

# Draw
draw.draw_corners(frame, corners)

cv2.imshow('frame', frame)

cv2.waitKey(0)