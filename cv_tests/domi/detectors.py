import numpy as np
import cv2
import cv2.cv as cv

def img_to_contours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    pips = []
    edges = []
    mask = np.zeros(img.shape, dtype=img.dtype)
    
    for i, contour in enumerate(contours):
    
        # Filter based on length and position of contour
        l = len(contour)
        if l < 20:
            # PIPS ARE UNTESTED
            pips.append(contour)
        else:
            is_border = False
            for point in contour:
                x = point[0][0]
                y = point[0][1]
                if x < 10 or x > img.shape[1] - 10 or y < 10 or y > img.shape[0] - 10:
                    is_border = True
                    break
            if not is_border:
                edges.append(contour)
                cv2.drawContours(mask, contours, i, 255, thickness=cv.CV_FILLED)

    return (pips, edges, mask)

def img_to_contour_points(img):
    pips_, edges_, mask = img_to_contours(img)

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

    return pips, edges, mask

def contour_point_to_angle(contour, i, k=1):
    x1 = contour[i][0]
    y1 = contour[i][1]
    x2 = contour[i-k][0]
    y2 = contour[i-k][1]
    x3 = contour[(i+k) % len(contour)][0]
    y3 = contour[(i+k) % len(contour)][1]
    
    dot = (x1 - x2) * (x3 - x1) + (y1 - y2) * (y3 - y1)
    cross = (x1 - x2) * (y3 - y1) - (y1 - y2) * (x3 - x1)
    d12 = (x1 - x2)**2 + (y1 - y2)**2
    d13 = (x1 - x3)**2 + (y1 - y3)**2
    
    denom = float(d12 * d13)
    if denom == 0:
        return 0

    val = float(dot) / np.sqrt(denom)
    angle = np.arccos(val)
    if cross < 0:
        angle *= -1

    return angle

def contour_points_to_corners(contour, k=1, radius=10):

    # First find all candidate corners according to the
    # angles between nearby points
    corners = []
    for i in xrange(0, len(contour)):
        if abs(contour_point_to_angle(contour, i, k=k)) > np.pi * 0.2:
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

def img_corners_to_groups(img, corners, margin=0.45):

    # Split corners into groups based on y position
    height = img.shape[0]
    stash = []
    game = []
    other_stash = []
    
    for contour in corners:
        if len(contour) == 0:
            continue
        corner = contour[0]
        if corner[1] < height * margin:
            stash.append(contour)
        elif corner[1] > height * (1 - margin):
            other_stash.append(contour)
        else:
            game.append(contour)

    assert (len(game) <= 1), "Too many contours found in the middle of the image!"

    return stash, game, other_stash

def corners_to_grid(corners, size):
    grid = {}
    
    x_grid = 0
    y_grid = 0
    grid[(0,0)] = corners[0]
    
    dx_grid = -1
    dy_grid = 0
    
    for i in xrange(0, len(corners)):
        x = corners[i][0]
        y = corners[i][1]
        x_prev = corners[i-1][0]
        y_prev = corners[i-1][1]
        
        dx = x - x_prev
        dy = y - y_prev
        
        length = np.round(np.sqrt(dx**2 + dy**2) / size)
        if length == 0:
            length = 1
        dx /= length
        dy /= length
        
        d_angle = np.round(contour_point_to_angle(corners, i-1) / (np.pi / 2))
        if d_angle == 1:
            # Rotate dx_grid, dy_grid 90 deg CW
            temp = dx_grid
            dx_grid = dy_grid
            dy_grid = -temp
        elif d_angle == -1:
            # Rotate dx_grid, dy_grid 90 deg CCW
            temp = dx_grid
            dx_grid = -dy_grid
            dy_grid = temp
            
        
        for i in xrange(1, int(length) + 1):
            x_grid += dx_grid
            y_grid += dy_grid
            x_img = int(x_prev + dx * i)
            y_img = int(y_prev + dy * i)
            grid[(x_grid, y_grid)] = (x_img, y_img)

    assert (x_grid == 0 and y_grid == 0), "Grid not aligned!"

    return grid

def all_corners_to_grids(corners, size):
    grids = []
    for contour in corners:
        grids.append(corners_to_grid(contour, size))

    return grids

def grid_to_connector_positions(grid, mask):

    # Find all positions that are the bottom left corner of a piece
    piece_positions = set()
    for p in grid:
    
        # Check that the other corners exist in the grid
        up = grid.get((p[0], p[1] + 1))
        right = grid.get((p[0] + 1, p[1]))
        diag = grid.get((p[0] + 1, p[1] + 1))
        if up != None and right != None and diag != None:
        
            # And that the center if the piece is within the contour
            x = int((grid[p][0] + up[0] + right[0] + diag[0]) / 4)
            y = int((grid[p][1] + up[1] + right[1] + diag[1]) / 4)
            if mask[y,x] > 0:
                piece_positions.add(p)

    # Now find the positions that only have another piece in one direction
    # These are the ends
    connectors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    for p in piece_positions:
        adj_dx = None
        adj_dy = None
        for dx, dy in directions:
            if (p[0] + dx, p[1] + dy) in piece_positions:
                if adj_dx == None:
                    adj_dx = dx
                    adj_dy = dy
                else:
                    adj_dx = None
                    break
        if adj_dx != None:
            connectors.append((p, (-adj_dx, -adj_dy)))

    return connectors

def count_pips_in_grid_position(grid, p, pip_contours):

    # Create piece boundary from grid
    p0 = np.array(grid[p])
    p1 = np.array(grid[(p[0], p[1] + 1)])
    p2 = np.array(grid[(p[0] + 1, p[1] + 1)])
    p3 = np.array(grid[(p[0] + 1, p[1])])
    contour = np.array([p0, p1, p2, p3])

    # Count pip contours that lie within this boundary
    count = 0
    for pip_contour in pip_contours:
        if cv2.pointPolygonTest(contour, pip_contour[0], False) >= 0:
            count += 1

    return count

def grid_point_to_pos_and_ori(grid, point, direction):

    # Choose p1 and p2 such that they go CCW on the edge in the give direction
    p1 = None
    p2 = None
    if direction[0] == 1:
        p1 = (point[0] + 1, point[1])
        p2 = (point[0] + 1, point[1] + 1)
    elif direction[0] == -1:
        p1 = (point[0], point[1] + 1)
        p2 = point
    elif direction[1] == 1:
        p1 = (point[0] + 1, point[1] + 1)
        p2 = (point[0], point[1] + 1)
    elif direction[1] == -1:
        p1 = point
        p2 = (point[0] + 1, point[1])

    pos1 = grid[p1]
    pos2 = grid[p2]
    pos = ((pos1[0] + pos2[0]) / 2, (pos1[1] + pos2[1]) / 2)
    ori = np.arctan2(pos1[1] - pos2[1], pos2[0] - pos1[0]) - np.pi / 2

    return pos, ori

def grid_to_connectors(grid, mask, pip_contours):
    connectors = grid_to_connector_positions(grid, mask)
    
    full_connectors = []
    for point, direction in connectors:
        pip_count = count_pips_in_grid_position(grid, point, pip_contours)
        pos, ori = grid_point_to_pos_and_ori(grid, point, direction)
        full_connectors.append((pip_count, pos, ori))
    
    return full_connectors

def all_grids_to_connectors(grids, mask, pip_contours):
    connectors = []
    for grid in grids:
        connectors.append(grid_to_connectors(grid, mask, pip_contours))

    return connectors




