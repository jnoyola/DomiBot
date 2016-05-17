import numpy as np
import cv2

def calibrate_with_corners(corners, should_print=False):
    total_short_edge_len = 0
    num_used = 0
    for domino in corners:
        if len(domino) != 4:
            continue
        
        dists = []
        
        x_prev = domino[-1][0]
        y_prev = domino[-1][1]
        for corner in domino:
            x = corner[0]
            y = corner[1]
            
            dists.append((x - x_prev)**2 + (y - y_prev)**2)

            x_prev = x
            y_prev = y

        dists.sort()
        total_short_edge_len += np.sqrt(dists[0])
        total_short_edge_len += np.sqrt(dists[1])
        num_used += 1

    assert (num_used > 0), "No individual dominoes detected for calibration!"
    
    short_edge_len = total_short_edge_len / (2 * num_used)
    
    # Multiply by 0.5 long:short edge ratio
    short_edge_len *= 1.06

    if should_print:
        print "Calibration: using " + str(num_used) + " individual dominoes"
        print "Calibration: short_edge_len = " + str(short_edge_len)
    return short_edge_len

def cam_to_world_point(frame, point):
    x = point[0][0]
    y = point[0][1]
    ori = point[1]

    x = 0 + x * 1 / frame.shape[1]
    y = 0 + y * 1 / frame.shape[0]

    return ((x, y), ori)