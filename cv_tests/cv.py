import numpy as np
import cv2
import cv2.cv as cv
from domi import calibration, filters, detectors, draw

# Load camera feed
#cap = cv2.VideoCapture(0)
#ret, frame = cap.read()

# Load image
frame = cv2.imread('frame.png')


# Filter
black_white = filters.black_white(frame)

# Detect
pip_contours, edge_contours = detectors.img_to_contour_points(black_white)
corners = detectors.all_contour_points_to_corners(edge_contours)

# Calibrate
size = calibration.calibrate_with_corners(corners)

# Compute
stash_corners, game_corners, other_stash_corners = detectors.img_corners_to_groups(frame, corners)
stash_grids = detectors.all_corners_to_grids(stash_corners, size)
game_grids = detectors.all_corners_to_grids(game_corners, size)

# Draw
draw.draw_grids(frame, stash_grids, color=(255,0,0))
draw.draw_grids(frame, game_grids, color=(0,255,0))

cv2.imshow('frame', frame)

cv2.waitKey(0)