import numpy as np
import cv2

def img_to_contours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    pips = []
    edges = []
    
    for contour in contours:
    
        # Filter based on length and position of contour
        l = len(contour)
        if l < 30:
            # PIPS ARE UNTESTED
            pips.append(contour)
        else:
            is_border = False
            for point in contour:
                x = point[0][0]
                if x < 10:
                    is_border = True
                    break
            if not is_border:
                edges.append(contour)

    return (pips, edges)

def img_to_contour_points(img):
    pips_, edges_ = img_to_contours(img)

    pips = []
    for contour in pips_:
        contour_points = []
        for point in contour:
            contour_points.append((point[0][0], point[0][1]))
        pips.append(contour_points)
    
    edges = []
    for contour in edges_:
        contour_points = []
        for point in contour:
            contour_points.append((point[0][0], point[0][1]))
        edges.append(contour_points)

    return pips, edges

def contour_point_to_angle(contour, i, k=3):
    x1 = contour[i][0]
    y1 = contour[i][1]
    x2 = contour[i-k][0]
    y2 = contour[i-k][1]
    x3 = contour[(i+k) % len(contour)][0]
    y3 = contour[(i+k) % len(contour)][1]
    
    dot = (x1 - x2) * (x3 - x1) + (y1 - y2) * (y3 - y1)
    d12 = (x1 - x2)**2 + (y1 - y2)**2
    d13 = (x1 - x3)**2 + (y1 - y3)**2
    
    denom = float(d12 * d13)
    if denom == 0:
        return 0

    val = float(dot) / np.sqrt(denom)
    angle = np.arccos(val)

    return angle

def contour_points_to_corners(contour, k=3, radius=12):

    # First find all candidate corners according to the
    # angles between nearby points
    corners = []
    for i in xrange(0, len(contour)):
        if abs(contour_point_to_angle(contour, i, k=k)) > np.pi * 0.3:
            corners.append(contour[i])

    # Now filter out corners by collecting groups/chains
    # of nearby corners and averaging them
    filtered_corners = []
    while len(corners) > 0:
    
        near = {corners.pop(0)}
        
        while True:
            # Look for a corner near one we already have in the set
            new_corner = None
            for corner in corners:
                for other in near:
                    d = (corner[0] - other[0])**2 + (corner[1] - other[1])**2
                    if d < radius**2:
                        new_corner = corner
                        break
                        
                if new_corner != None:
                    break
        
            # If we didn't find one, finish. Otherwise add it and continue
            if new_corner == None:
                break
            else:
                corners.remove(new_corner)
                near.add(new_corner)

        # Average the positions of all nearby corners
        x = 0
        y = 0
        for corner in near:
            x += corner[0]
            y += corner[1]

        x /= len(near)
        y /= len(near)
        filtered_corners.append((x,y))

    return filtered_corners

def all_contour_points_to_corners(contours, k_list=[3,1]):
    corners = []
    for contour in contours:
        for k in k_list:
            contour = contour_points_to_corners(contour, k=k)
        corners.append(contour)
        
    return corners
