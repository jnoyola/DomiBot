import cv2
import numpy as np
from domi import calibration, filters, detectors, strategy, draw, write

# Load camera feed
cap = cv2.VideoCapture(0)
ret, frame = cap.read()

# # Load image
# frame = cv2.imread('frame.png')


# Filter
black_white = filters.black_white(frame)

# Detect contours
pips, edges, mask = detectors.img_to_contours(black_white)

# Split groups
stash_contours, game_contours, other_stash_contours = detectors.contours_to_groups(frame, edges)

# Get bounding boxes
stash_rects = detectors.contours_to_rects(stash_contours)
game_rects = detectors.contours_to_rects(game_contours)

# Calibrate domino size
size = calibration.calibrate_with_rects(stash_rects)

# Find bounding boxes for the ends of every domino/train
stash_end_rects = detectors.all_contours_to_end_rects(stash_contours, stash_rects, size)
game_end_rects = detectors.all_contours_to_end_rects(game_contours, game_rects, size)

# Find the extreme ends of the train
train_end_rects = detectors.end_rects_to_extreme_end_rects(game_end_rects)

# Count pips and orientation for each end
stash_connectors = detectors.all_end_rects_to_connectors(stash_end_rects, mask, pips)
train_connectors = detectors.all_end_rects_to_connectors(train_end_rects, mask, pips)

# Strategy
des_domino, des_get, des_put = strategy.desired_targets(stash_connectors, train_connectors[0], size + 12)

# Draw
# draw.draw_all_rects(frame, stash_end_rects)
# draw.draw_all_rects(frame, train_end_rects)
# draw.draw_partitions(frame)
# draw.draw_connectors(frame, stash_connectors)
# draw.draw_connectors(frame, train_connectors)

# if des_domino[0] != -1:
# 	draw.draw_pos_ori(frame, des_get)
# 	draw.draw_pos_ori(frame, des_put)

# cv2.imshow('frame', frame)
# has_error = cv2.waitKey(0) == ord('q')
# write.write_file(frame, has_error=has_error)

# Transform
if des_domino[0] != -1:
	des_get = calibration.cam_to_world_point(frame, des_get)
	des_put = calibration.cam_to_world_point(frame, des_put)

# # Write
write.write_desired_play(des_domino, des_get, des_put)