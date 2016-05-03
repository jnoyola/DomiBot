import numpy as np
import math
import time
import cv2
import cv2.cv as cv

contour = [[[894, 444]], \
           [[895, 443]], \
           [[901, 443]], \
           [[902, 444]], \
           [[904, 444]], \
           [[905, 445]], \
           [[908, 445]], \
           [[909, 446]], \
           [[911, 446]], \
           [[912, 447]], \
           [[915, 447]], \
           [[916, 448]], \
           [[917, 448]], \
           [[918, 449]], \
           [[918, 454]], \
           [[917, 455]], \
           [[917, 457]], \
           [[916, 458]], \
           [[916, 460]], \
           [[915, 461]], \
           [[915, 463]], \
           [[914, 464]], \
           [[914, 466]], \
           [[913, 467]], \
           [[913, 471]], \
           [[912, 472]], \
           [[912, 473]], \
           [[911, 474]], \
           [[911, 477]], \
           [[910, 478]], \
           [[910, 480]], \
           [[909, 481]], \
           [[909, 483]], \
           [[908, 484]], \
           [[908, 486]], \
           [[907, 487]], \
           [[907, 490]], \
           [[906, 491]], \
           [[906, 493]], \
           [[905, 494]], \
           [[905, 496]], \
           [[903, 498]], \
           [[901, 498]], \
           [[900, 497]], \
           [[897, 497]], \
           [[896, 496]], \
           [[895, 496]], \
           [[894, 495]], \
           [[890, 495]], \
           [[889, 494]], \
           [[887, 494]], \
           [[886, 493]], \
           [[884, 493]], \
           [[883, 492]], \
           [[882, 492]], \
           [[880, 490]], \
           [[880, 486]], \
           [[881, 485]], \
           [[882, 485]], \
           [[882, 483]], \
           [[881, 482]], \
           [[882, 481]], \
           [[882, 480]], \
           [[883, 479]], \
           [[883, 477]], \
           [[884, 476]], \
           [[884, 475]], \
           [[885, 474]], \
           [[885, 471]], \
           [[886, 470]], \
           [[886, 467]], \
           [[887, 466]], \
           [[887, 463]], \
           [[889, 461]], \
           [[889, 456]], \
           [[890, 455]], \
           [[890, 454]], \
           [[891, 453]], \
           [[891, 450]], \
           [[892, 449]], \
           [[892, 448]], \
           [[893, 447]], \
           [[893, 446]], \
           [[894, 445]]]

def get_angle_for_point(contour, i, k=3):
    x1 = contour[i][0][0]
    y1 = contour[i][0][1]
    x2 = contour[i-k][0][0]
    y2 = contour[i-k][0][1]
    x3 = contour[(i+k) % len(contour)][0][0]
    y3 = contour[(i+k) % len(contour)][0][1]
    
    dot = (x1 - x2) * (x3 - x1) + (y1 - y2) * (y3 - y1)
    d12 = (x1 - x2)**2 + (y1 - y2)**2
    d13 = (x1 - x3)**2 + (y1 - y3)**2

    val = float(dot) / np.sqrt(float(d12 * d13))
    angle = np.arccos(val)
    return angle

def get_corners(contour):
    corners = []
    twice_corner_index = -1
    
    # Find corners by angle
    # Then filter out consecutive detected corner points
    # by taking the middle one
    
    for i in xrange(0, len(contour)):
        if abs(get_angle_for_point(contour, i)) > np.pi * 0.25:
            if twice_corner_index < 0:
                twice_corner_index = 2 * i
            else:
                twice_corner_index += 1
        else:
            if twice_corner_index > 0:
                corners.append(twice_corner_index / 2)
                twice_corner_index = -1

    return corners


frame = cv2.imread('frame.png')

x_m = frame.shape[1]
y_m = frame.shape[0]
x_M = 0
y_M = 0

corner_idxs = get_corners(contour)
for i in corner_idxs:
    x = contour[i][0][0]
    y = contour[i][0][1]
    cv2.circle(frame, (x, y), 2, (0, 255, 0))
    if x < x_m:
        x_m = x
    if x > x_M:
        x_M = x
    if y < y_m:
        y_m = y
    if y > y_M:
        y_M = y

frame = frame[y_m - 10:y_M + 11, x_m - 10:x_M + 11]
frame= cv2.resize(frame, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)

cv2.imshow('frame', frame)

cv2.waitKey(0)