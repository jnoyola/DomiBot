import numpy as np
import cv2
import cv2.cv as cv

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= - np.pi:
        angle += 2 * np.pi
    return angle

def angle_diff(angle0, angle1):
    diff = abs(angle1 - angle0)
    if diff > np.pi:
        diff = 2 * np.pi - diff
    return diff

def img_to_contours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    pips = []
    edges = []
    mask = np.zeros(img.shape, dtype=img.dtype)
    margin = 5
    
    for i, contour in enumerate(contours):
    
        # Filter based on length and position of contour
        l = len(contour)
        if l < 10:
            pips.append(contour)
        else:
            is_border = False
            for point in contour:
                x = point[0][0]
                y = point[0][1]
                if x < margin or x > img.shape[1] - margin or y < margin or y > img.shape[0] - margin:
                    is_border = True
                    break
            if not is_border:
                edges.append(contour)
                cv2.drawContours(mask, contours, i, 255, thickness=cv.CV_FILLED)

    return (pips, edges, mask)

def contours_to_groups(img, contours, margin=0.33):

    # CHANGE THIS TO 3-MEANS CLUSTERING
    # OR AT LEAST USE CONTOUR CENTROIDS

    # Split corners into groups based on y position
    height = img.shape[0]
    stash = []
    game = []
    other_stash = []
    
    for contour in contours:
        if len(contour) == 0:
            continue
        y = contour[0][0][1]
        if y < height * margin:
            stash.append(contour)
        elif y > height * (1 - margin):
            other_stash.append(contour)
        else:
            game.append(contour)

    return stash, game, other_stash

def contours_to_rects(contours):
    rects = []
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        rect = cv2.cv.BoxPoints(rect)
        rects.append(rect)

    return rects

def get_closest_point_in_contour(contour, p):
    # Brute force: expand radially outwards until we find a point in the contour
    r = 0.0
    while True:
        for i in range(64):
            x = int(p[0] + r * np.cos(i * np.pi / 32))
            y = int(p[1] + r * np.sin(i * np.pi / 32))
            if cv2.pointPolygonTest(contour, ((x,y)), False) >= 0:
                return (x,y)
        r += 1

def contour_to_end_rects(contour, rect, size):
    end_rects = [[], []]
    end_idx = 0

    x0, y0 = get_closest_point_in_contour(contour, rect[-1])

    for p in rect:
        x1, y1 = get_closest_point_in_contour(contour, p)

        d_2 = (x1-x0)**2 + (y1-y0)**2

        if d_2 < (size * 1.5)**2:
            # This edge is a single domino width
            # Append this corner to our current end
            end_rects[end_idx].append((x1, y1))
        elif d_2 < (size * 2.5)**2:
            # This edge is a full domino length
            # Append the midpoint to both ends
            # Then switch the current end; we're now working on the other end
            # Finally add the current point to the new current end
            p = ((x0 + x1) / 2, (y0 + y1) / 2)
            p = get_closest_point_in_contour(contour, p)

            end_rects[end_idx].append(p)

            end_idx = 1 - end_idx

            end_rects[end_idx].append(p)
            end_rects[end_idx].append((x1, y1))
        else:
            # This edge is the long edge of a multi-domino train
            # Add the point that is a single domino width along this edge to the current end
            # Then switch the current end; we're now working on the other end
            # Next add the point that is a single domino width back along this edge to the new current end
            # Finally add the current point to the new current end
            ratio = size / np.sqrt(d_2)
            p = (x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio)
            p = get_closest_point_in_contour(contour, p)
            end_rects[end_idx].append(p)

            end_idx = 1 - end_idx

            p = (x1 - (x1 - x0) * ratio, y1 - (y1 - y0) * ratio)
            p = get_closest_point_in_contour(contour, p)
            end_rects[end_idx].append(p)
            end_rects[end_idx].append((x1, y1))

        x0 = x1
        y0 = y1

    return end_rects

def all_contours_to_end_rects(contours, rects, size):
    all_end_rects = []
    for i in xrange(len(contours)):
        all_end_rects.append(contour_to_end_rects(contours[i], rects[i], size))

    return all_end_rects

def end_rects_to_extreme_end_rects(end_rects):
    # Find the end rects with min x and max x
    x_min = 100000
    x_max = 0
    rect_min = None
    rect_max = None

    for ends in end_rects:
        for rect in ends:
            for p in rect:
                x = p[0]
                if x < x_min:
                    x_min = x
                    rect_min = rect
                if x > x_max:
                    x_max = x
                    rect_max = rect

    return ((rect_min, rect_max),)

def end_rect_to_closest_outward_angle_and_point(end_rect, angle):
    # Get the outward facing angle of each edge
    # and find the one closest to the given angle
    # Then average the two points from this edge
    closest_angle = 0
    closest_points = None
    min_diff = 1000

    x0, y0 = end_rect[-1]
    for p in end_rect:
        x1 = p[0]
        y1 = p[1]
        theta = np.arctan2(y1 - y0, x1 - x0) - np.pi / 2
        theta = normalize_angle(theta)

        diff = angle_diff(angle, theta)
        if diff < min_diff:
            min_diff = diff

            # Negate here to switch to the right-handed system
            closest_angle = -theta

            closest_points = (x0, x1, y0, y1)

        x0 = x1
        y0 = y1

    p = ((closest_points[0] + closest_points[1]) / 2, (closest_points[2] + closest_points[3]) / 2)

    return (closest_angle, p)

def end_rects_to_opposing_orientations_and_points(end_rects):
    # Find center of each end rect
    x0 = 0.0
    y0 = 0.0
    x1 = 0.0
    y1 = 0.0
    for p in end_rects[0]:
        x0 += p[0] / 4
        y0 += p[1] / 4
    for p in end_rects[1]:
        x1 += p[0] / 4
        y1 += p[1] / 4

    angle0 = np.arctan2(y0 - y1, x0 - x1)
    angle1 = angle0 + np.pi

    angle0 = normalize_angle(angle0)
    angle1 = normalize_angle(angle1)

    angle0, p0 = end_rect_to_closest_outward_angle_and_point(end_rects[0], angle0)
    angle1, p1 = end_rect_to_closest_outward_angle_and_point(end_rects[1], angle1)

    return (angle0, angle1, p0, p1)


def rect_to_pip_count(rect, pip_contours):
    # Convert rect into a contour
    p0 = np.array(rect[0])
    p1 = np.array(rect[1])
    p2 = np.array(rect[2])
    p3 = np.array(rect[3])
    contour = np.array([p0, p1, p2, p3])

    # Count pip contours that lie within this rect
    count = 0
    for pip_contour in pip_contours:
        p = (pip_contour[0][0][0], pip_contour[0][0][1])
        if cv2.pointPolygonTest(contour, p, False) >= 0:
            count += 1

    return count

def end_rects_to_connectors(end_rects, mask, pip_contours):
    angle0, angle1, p0, p1 = end_rects_to_opposing_orientations_and_points(end_rects)

    pips0 = rect_to_pip_count(end_rects[0], pip_contours)
    pips1 = rect_to_pip_count(end_rects[1], pip_contours)

    return [(pips0, p0, angle0), (pips1, p1, angle1)]

def all_end_rects_to_connectors(all_end_rects, mask, pip_contours):
    all_connectors = []
    for end_rects in all_end_rects:
        all_connectors.append(end_rects_to_connectors(end_rects, mask, pip_contours))

    return all_connectors
