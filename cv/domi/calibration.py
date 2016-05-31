import numpy as np
import cv2

def calibrate_with_rects(rects):
    total_short_edge_len = 0
    num_used = 0
    for rect in rects:

        # Compute distances between each adjacent pair of points
        dists = []
        
        x0 = rect[-1][0]
        y0 = rect[-1][1]
        for corner in rect:
            x1 = corner[0]
            y1 = corner[1]
            
            dists.append((x1 - x0)**2 + (y1 - y0)**2)

            x0 = x1
            y0 = y1

        # Sort dists to get the two short edges of the domino
        dists.sort()
        total_short_edge_len += np.sqrt(dists[0])
        total_short_edge_len += np.sqrt(dists[1])
        num_used += 1

    assert (num_used > 0), "No individual dominoes detected for calibration!"
    
    short_edge_len = total_short_edge_len / (2 * num_used)
    
    # Manual adjustment
    short_edge_len *= 0.85

    return short_edge_len

def cam_to_world_point(frame, point):
    x = point[0][0]
    y = point[0][1]
    ori = point[1]

    K = np.matrix('823.93153339    0.          337.75817372;' \
                  '   0.          825.08985445  233.26352724;' \
                  '   0.            0.            1.        ')
    distortion = np.matrix('-0.04679004  0.40376876 -0.00542364  0.00363889 -0.63768937')

    p = np.zeros((1,1,2), dtype=np.float32)
    p[0,0,0] = y
    p[0,0,1] = x
    p = cv2.undistortPoints(p, K, distortion, P=K)
    y = p[0,0,0]
    x = p[0,0,1]

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

    M = np.matrix(' -8.28075020e+02   6.47791220e+00   3.27404267e+02   2.97028663e+02;' \
                  ' -1.17633761e+01  -8.26684099e+02   2.27244723e+02   1.74052476e+02;' \
                  ' -1.26008059e-02  -7.13016553e-03   9.99895185e-01   7.11481742e-01')

    x_new =  (M[0,1]*M[1,3] - M[0,3]*M[1,1] + (M[1,1]*M[2,3] - M[1,3]*M[2,1])*x + (M[0,3]*M[2,1] - M[0,1]*M[2,3])*y)
    y_new = -(M[0,0]*M[1,3] - M[0,3]*M[1,0] + (M[1,0]*M[2,3] - M[1,3]*M[2,0])*x + (M[0,3]*M[2,0] - M[0,0]*M[2,3])*y)
    w_new =  (M[0,0]*M[1,1] - M[0,1]*M[1,0] + (M[1,0]*M[2,1] - M[1,1]*M[2,0])*x + (M[0,1]*M[2,0] - M[0,0]*M[2,1])*y)

    x_new = 0.1430 - x_new/w_new
    y_new = -0.6112 + y_new/w_new

    return ((x_new, y_new), ori)
