import cv2
import glob

# Load camera feed
cap = cv2.VideoCapture(0)
ret, frame = cap.read()

images = glob.glob('*.png')

cv2.imwrite('frame' + str(len(images)) + '.png', frame)
