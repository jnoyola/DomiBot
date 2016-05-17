import cv2

# Load camera feed
cap = cv2.VideoCapture(0)
ret, frame = cap.read()


cv2.imwrite('frame0.png', frame)