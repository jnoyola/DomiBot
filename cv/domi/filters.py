import cv2

def black_white(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, out = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY)
    return out