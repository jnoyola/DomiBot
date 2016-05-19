import os
import numpy as np
import cv2

def write_file(img, has_error):
    dir = os.path.dirname(os.path.dirname(__file__)) + "/frames/"
    
    file_count = 0
    if os.path.exists(dir):
        path, dirs, files = os.walk(dir).next()
        file_count = len(files)
    else:
        os.makedirs(dir)
    
    filepath = dir + "frame_" + str(file_count)
    if has_error:
        filepath += "_error"
    filepath += ".png"
    cv2.imwrite(filepath, img)

def write_desired_play(des_domino, des_get, des_put):
    print str(des_domino[0]) + ',' + \
          str(des_domino[1]) + ',' + \
          str(des_get[0][0]) + ',' + \
          str(des_get[0][1]) + ',' + \
          str(des_get[1]) + ',' + \
          str(des_put[0][0]) + ',' + \
          str(des_put[0][1]) + ',' + \
          str(des_put[1])