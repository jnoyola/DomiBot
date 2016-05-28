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

    # w = frame.shape[1]
    # h = frame.shape[0]

    # # Camera and World coordinates
    # x_c = [     0,      w,        0]
    # y_c = [     0,      0, h*0.8052]
    # # x_w = [-0.1696,  0.382,        -0.152]
    # # y_w = [ -0.447, -0.415,        -0.776]
    # x_w = [-0.170,  0.379,   -0.153]
    # y_w = [-0.447, -0.386,   -0.764]

    # x_new = x_w[0] \
    #       + (x_w[1] - x_w[0]) * (x - x_c[0]) / (x_c[1] - x_c[0]) \
    #       + (x_w[2] - x_w[0]) * (y - y_c[0]) / (y_c[2] - y_c[0])
    # y_new = y_w[0] \
    #       + (y_w[1] - y_w[0]) * (x - x_c[0]) / (x_c[1] - x_c[0]) \
    #       + (y_w[2] - y_w[0]) * (y - y_c[0]) / (y_c[2] - y_c[0])

    M = np.matrix('-8.57100170e+02   2.38442982e+01   3.25323825e+02   3.07510401e+02;' \
                  '-1.29961914e+01  -8.55575310e+02   2.21193764e+02   1.83634546e+02;' \
                  ' 1.31384576e-02   2.52420606e-02   9.99595028e-01   7.42007236e-01')

    x_new =  (M[0,1]*M[1,3] - M[0,3]*M[1,1] + (M[1,1]*M[2,3] - M[1,3]*M[2,1])*x + (M[0,3]*M[2,1] - M[0,1]*M[2,3])*y)
    y_new = -(M[0,0]*M[1,3] - M[0,3]*M[1,0] + (M[1,0]*M[2,3] - M[1,3]*M[2,0])*x + (M[0,3]*M[2,0] - M[0,0]*M[2,3])*y)
    w_new =  (M[0,0]*M[1,1] - M[0,1]*M[1,0] + (M[1,0]*M[2,1] - M[1,1]*M[2,0])*x + (M[0,1]*M[2,0] - M[0,0]*M[2,1])*y)

    x_new = 0.1444 - x_new/w_new
    y_new = -0.6186 + y_new/w_new

    return ((x_new, y_new), ori)
