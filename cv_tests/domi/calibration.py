import numpy as np
import cv2

def calibrate_with_corners(corners):
    print "Calibrating with " + str(len(corners)) + " dominoes."

    total_short_edge_len = 0
    for domino in corners:
        assert (len(domino) > 3), "Not enough corners detected for calibration!"
        assert (len(domino) < 5), "Too many corners detected for calibration!"
        
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
        
    short_edge_len = total_short_edge_len / (2 * len(corners))

    print "Calibration: short_edge_len = " + str(short_edge_len)
    return short_edge_len