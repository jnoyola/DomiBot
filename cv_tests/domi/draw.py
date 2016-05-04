import cv2

def draw_corners(img, corners, r=2, color=(0,255,0), thickness=2):
    for arr in corners:
        for corner in arr:
            cv2.circle(img, (corner[0], corner[1]), r, color, thickness)

def draw_grids(img, grids, r=2, color=(0,255,0), thickness=2):
    for grid in grids:
        for point in grid.itervalues():
            cv2.circle(img, point, r, color, thickness)