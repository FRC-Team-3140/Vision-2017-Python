import cv2
import numpy as np
import argparse

parser = argparse.ArgumentParser(description="Captures Video from Bot Cameras")
parser.add_argument('--debug', default=False, action='store_const', const=True, help='Debug Mode')
args=parser.parse_args()

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

id = 0

camera = cv2.VideoCapture(id)

while(camera.isOpened()):
	# Capture frame-by-frame
	ret, frame = camera.read()
	if ret==True:
		out.write(frame)
		cv2.imshow('frame',frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		else:
			break

# When everything done, release the capture
camera.release()
out.release()
cv2.destroyAllWindows()