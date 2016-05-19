import numpy as np
import cv2

def draw_corners(img, corners, r=2, color=(0,255,0), thickness=2):
    for arr in corners:
        for corner in arr:
            cv2.circle(img, (corner[0], corner[1]), r, color, thickness)

def draw_grids(img, grids, r=2, color=(0,255,0), thickness=2):
    for grid in grids:
        for point in grid.itervalues():
            cv2.circle(img, point, r, color, thickness)

def draw_partitions(img, margin=0.33, color=(0,0,0), thickness=2):
    height = img.shape[0]
    y_top = int(height * margin)
    y_bot = int(height * (1 - margin))
    cv2.line(img, (0, y_top), (img.shape[1] - 1, y_top), color, thickness)
    cv2.line(img, (0, y_bot), (img.shape[1] - 1, y_bot), color, thickness)

def draw_connectors(img, connectors, length=5, color=(0,0,150), thickness=2):
    for component in connectors:
        for pips, pos, ori in component:
            x2 = int(pos[0] + length * np.cos(-ori))
            y2 = int(pos[1] + length * np.sin(-ori))
            cv2.line(img, pos, (x2, y2), color, thickness)
            
            x3 = int(x2 + 10 * np.cos(-ori)) - 5
            y3 = int(y2 + 10 * np.sin(-ori)) + 5
            cv2.putText(img, str(pips), (x3, y3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

def draw_pos_ori(img, point, r=2, length=10, color=(0,255,0), thickness=2):
    x = int(point[0][0])
    y = int(point[0][1])
    ori = point[1]
    
    x2 = int(x + length * np.cos(-ori))
    y2 = int(y + length * np.sin(-ori))

    cv2.circle(img, (x, y), r, color, thickness)
    cv2.line(img, (x, y), (x2, y2), color, thickness)