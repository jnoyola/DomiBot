import numpy as np
import cv2
import glob

def cam_to_world_point(M,u,v):
    # Assume z = 0
    x =  (M[0,1]*M[1,3] - M[0,3]*M[1,1] + (M[1,1]*M[2,3] - M[1,3]*M[2,1])*u + (M[0,3]*M[2,1] - M[0,1]*M[2,3])*v)
    y = -(M[0,0]*M[1,3] - M[0,3]*M[1,0] + (M[1,0]*M[2,3] - M[1,3]*M[2,0])*u + (M[0,3]*M[2,0] - M[0,0]*M[2,3])*v)
    w =  (M[0,0]*M[1,1] - M[0,1]*M[1,0] + (M[1,0]*M[2,1] - M[1,1]*M[2,0])*u + (M[0,1]*M[2,0] - M[0,0]*M[2,1])*v)
    return (x/w, y/w)

grid_width = 6
grid_height = 9

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((grid_width*grid_height,3), np.float32)
objp[:,:2] = np.mgrid[0:grid_height,0:grid_width].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (grid_height,grid_width),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
        imgpoints.append(corners)

        # Draw and display the corners
#        cv2.drawChessboardCorners(img, (grid_height,grid_width), corners,ret)
#        cv2.imshow('img',img)
#        cv2.waitKey(500)

cv2.destroyAllWindows()

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

R,J = cv2.Rodrigues(rvecs[-1])
M = K.dot(np.concatenate((R, tvecs[-1]), axis=1))
p = M.dot(np.array([0,0,0,1]))
print M
print cam_to_world_point(M,368,678)