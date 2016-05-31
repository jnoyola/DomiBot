import numpy as np
import cv2
import glob

def cam_to_world_point(K,dist,M,u,v,z=0):
	p = np.zeros((1,1,2), dtype=np.float32)
	p[0,0,0] = v
	p[0,0,1] = u
	p = cv2.undistortPoints(p, K, dist, P=K)
	v = p[0,0,0]
	u = p[0,0,1]

	m00 = M[0,0]
	m01 = M[0,1]
	m02 = M[0,2]
	m03 = M[0,3]
	m10 = M[1,0]
	m11 = M[1,1]
	m12 = M[1,2]
	m13 = M[1,3]
	m20 = M[2,0]
	m21 = M[2,1]
	m22 = M[2,2]
	m23 = M[2,3]
	x =  (m01*m13 - m03*m11 + m11*m23*u - m13*m21*u - m01*m23*v + m03*m21*v + m01*m12*m23*z - m01*m13*m22*z - m02*m11*m23*z + m02*m13*m21*z + m03*m11*m22*z - m03*m12*m21*z)
	y = -(m00*m13 - m03*m10 + m10*m23*u - m13*m20*u - m00*m23*v + m03*m20*v + m00*m12*m23*z - m00*m13*m22*z - m02*m10*m23*z + m02*m13*m20*z + m03*m10*m22*z - m03*m12*m20*z)
	w =  (m00*m11 - m01*m10 + m10*m21*u - m11*m20*u - m00*m21*v + m01*m20*v - m00*m11*m22*z + m00*m12*m21*z + m01*m10*m22*z - m01*m12*m20*z - m02*m10*m21*z + m02*m11*m20*z)
	# return (x/w, y/w)
	return (0.1933 - x/w, -0.6107 + y/w)

grid_width = 9
grid_height = 6

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((grid_height*grid_width,3), np.float32)
objp[:,:2] = np.mgrid[0:grid_width,0:grid_height].T.reshape(-1,2)
objp *= 0.0222

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.png')

for fname in images:
	img = cv2.imread(fname)
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	# Find the chess board corners
	ret, corners = cv2.findChessboardCorners(gray, (grid_width,grid_height),None)

	# If found, add object points, image points (after refining them)
	if ret == True:
		objpoints.append(objp)

		cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
		imgpoints.append(corners)

		# Draw and display the corners
		# cv2.drawChessboardCorners(img, (grid_height,grid_width), corners,ret)
		# cv2.circle(img, (corners[0][0][0], corners[0][0][1]), 10, (255,0,0), 5)
		# cv2.imshow('img',img)
		# cv2.waitKey(1000)
	else:
		print fname

cv2.destroyAllWindows()

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print "RMS: " + str(ret)

ref_idx = 1
R,J = cv2.Rodrigues(rvecs[ref_idx])
M = K.dot(np.concatenate((R, tvecs[ref_idx]), axis=1))

print K
print dist
print M
print cam_to_world_point(K,dist,M,266,98,z=0)

 #-0.2037 -0.8065