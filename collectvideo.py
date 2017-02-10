##################################################
# Program to capture camera video to a file
# Usage: python collectvideo 0
#    or: python collectvideo 1
#
# where 0 or 1 is the desired camera
#
# Note the camera is set for enhanced imaging
# of the FRC vision targets which is used
# in the target detection routines.  The sample
# video collected from this program can be used
# to generate test video for the target detection
# routines during software development.
##################################################


import cv2
import numpy as np
import argparse
from pdb import set_trace as br

parser = argparse.ArgumentParser(description="Captures Video from Bot Cameras")
parser.add_argument('id', type=int, default=0, help='Select Camera id 0 or 1')
parser.add_argument('--debug', default=False, action='store_const', const=True, help='Debug Mode')
args=parser.parse_args()
id = args.id


fourcc = cv2.VideoWriter_fourcc(*'IYUV')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

camera = cv2.VideoCapture(id)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
camera.set(cv2.CAP_PROP_BRIGHTNESS, 150) 
camera.set(cv2.CAP_PROP_CONTRAST, 10) 
camera.set(cv2.CAP_PROP_EXPOSURE,-11)

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