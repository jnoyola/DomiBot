import numpy as np
import math
import time
import cv2
import cv2.cv as cv



cap = cv2.VideoCapture(0)

def do_cv():
#while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, black_white = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

    # Dilation/Erosion
    #kernel = np.ones((3,3),np.uint8)
    #black_white = cv2.dilate(black_white, kernel, iterations = 1)
    #black_white = cv2.erode(black_white, kernel, iterations = 1)
    
    # Blob detection
    #blob_params = cv2.SimpleBlobDetector_Params()
    #blob_params.minArea = 400
    #detector = cv2.SimpleBlobDetector(blob_params)
    #keypoints = detector.detect(black_white)
    #black_white = cv2.drawKeypoints(black_white, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #print len(keypoints)
    
    # Find/Draw contours
    contours, hierarchy = cv2.findContours(black_white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Contours by length:
        # short: pips
        # medium: edges
        # other: noise
    #mask = np.zeros(black_white.shape, dtype=frame.dtype)
    for i in xrange(0, len(contours)):
        # Filter based on length of contour
        l = len(contours[i])
        if l > 30 and l < 100:
            #cv2.drawContours(mask, contours, i, 1, cv.CV_FILLED)
            cv2.drawContours(frame, contours, i, (0,0,255))

    # Multiply by mask
    # frame = np.multiply(frame, mask[:, :, np.newaxis])
    
    # or use Canny
    #edges = cv2.Canny(black_white, 50, 150, apertureSize = 3)
    
    #dst = cv2.cornerHarris(edges,5,3,0.2)
    #frame[dst>0.05*dst.max()]=[0,0,255]
    
    #lines = cv2.HoughLinesP(edges, 1, np.pi/180 / 45, 10, minLineLength=10, maxLineGap=1)
    #for x1,y1,x2,y2 in lines[0]:
    #    cv2.line(edges,(x1,y1),(x2,y2),(80,),2)

    cv2.imshow('frame', frame)

    while(True):
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

            
do_cv()