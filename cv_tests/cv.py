import cv2
from domi import calibration, filters, detectors, draw, write

# Load camera feed
#cap = cv2.VideoCapture(0)
#ret, frame = cap.read()

# Load image
frame = cv2.imread('frame.png')


# Filter
black_white = filters.black_white(frame)

# Detect
pip_contours, edge_contours, mask = detectors.img_to_contour_points(black_white)
corners = detectors.all_contour_points_to_corners(edge_contours)

# Calibrate
size = calibration.calibrate_with_corners(corners, should_print=False)

# Compute
stash_corners, game_corners, other_stash_corners = detectors.img_corners_to_groups(frame, corners)
stash_grids = detectors.all_corners_to_grids(stash_corners, size)
game_grids = detectors.all_corners_to_grids(game_corners, size)

stash_connectors = detectors.all_grids_to_connectors(stash_grids, mask, pip_contours)
game_connectors = detectors.all_grids_to_connectors(game_grids, mask, pip_contours)

# Strategy

# Draw
draw.draw_grids(frame, stash_grids, color=(255,0,0))
draw.draw_grids(frame, game_grids, color=(0,255,0))
draw.draw_partitions(frame)
draw.draw_connectors(frame, stash_connectors)
draw.draw_connectors(frame, game_connectors)

cv2.imshow('frame', frame)
has_error = cv2.waitKey(0) == ord('q')
write.write_file(frame, has_error=has_error)

# Write
write.write_connectors(stash_connectors)