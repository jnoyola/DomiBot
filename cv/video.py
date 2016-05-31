import cv2

def mouse_pos(event, x, y, flags, param):
	if event == cv2.EVENT_LBUTTONDOWN:
		print (float(x) / 2, float(y) / 2)

cv2.namedWindow('frame')
cv2.setMouseCallback('frame', mouse_pos)

# Load camera feed
cap = cv2.VideoCapture(0)

while True:
	ret, frame = cap.read()
	frame = cv2.imread('frame9.png')

	frame = cv2.resize(frame, None, fx=2, fy=2, interpolation = cv2.INTER_CUBIC)

	cv2.imshow('frame', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break