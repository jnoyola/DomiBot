import numpy as np
import math
import time
import cv2
import cv2.cv as cv



cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
#    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break