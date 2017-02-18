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
import time
from pdb import set_trace as br

parser = argparse.ArgumentParser(description="Captures Video from Bot Cameras")
parser.add_argument('--id', type=int, action='store', default=0, help='Select Camera id 0 or 1')
parser.add_argument('--debug', default=False, action='store_const', const=True, help='Debug Mode')
args=parser.parse_args()
id = args.id

xSize=640
ySize=480
#fourcc = cv2.VideoWriter_fourcc(*'IYUV')
fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (xSize, ySize))

camera = cv2.VideoCapture(id)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, xSize) 
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, ySize) 
camera.set(cv2.CAP_PROP_BRIGHTNESS, 50)
ret, frame = camera.read()				# workaround for broken Brightness setting

camera.set(cv2.CAP_PROP_CONTRAST, 10) 
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, -1)
camera.set(cv2.CAP_PROP_EXPOSURE,-100)
camera.set(cv2.CAP_PROP_BRIGHTNESS, 30)

#camera.set(cv2.CAP_PROP_GAIN,-100)
#camera.set(cv2.CAP_PROP_SETTINGS, -1);    #pop up the driver window 


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