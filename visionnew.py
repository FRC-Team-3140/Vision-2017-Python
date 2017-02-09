#!C:\Program Files\Anaconda2\python.exe

import cv2
import numpy as np
import time
import argparse
import socket

from pdb import set_trace as br

parser = argparse.ArgumentParser(description="Finds 2017 Vision Targets")
parser.add_argument('--debug', default=False, action='store_const', const=True, help='Debug Mode')
args=parser.parse_args()

UDP_IP = "127.0.0.1"


def initCamera(id = 0):
	camera = cv2.VideoCapture(id)
	
	# Now we can initialize the camera capture object with the cv2.VideoCapture class.
	# All it needs is the index to a camera port.

	#camera.set(cv2.CV_CAP_PROP_FRAME_WIDTH, 640)
	#camera.set(cv2.CV_CAP_PROP_FRAME_HEIGHT, 480)

	camera.set(3, 640)
	camera.set(4, 480)

	return camera


cameraLow = initCamera(0)
cameraHigh = initCamera(1)
	

# Target Definitions - for a High vision target (boiler) and Low target (Gear placement)
# Defined as attributes of rectangles and their expected interdependices with 
# each other and the background

###### High Target (Boiler)################################################################
## Boiler target has two rectangular reflective tapes mounted horizontally around the
## funnel.  They are parallel separated by 2 inches.  The top tape is 4 inches tall and
## the bottom tape is 2 inches tall.  The diameter of the funnel they circle is 15 inches.
## The tape won't reflect off axis so it will likely appear less than the full 15 inches.

targetHigh = {
	'NumRects' : 2,
	'Rects' : [[16.0,4.0],[16.0,2.0]], #inches width x height for both rectangles
	'RectSep' : [0.0,5.0], #inches width, height in separation between rectanglestargetHighRectSep,
	'RectIntensity' : [True,True], #each rectangle should be brighter than surrounding
	'RectSepTol' : 0.25, #inches tolernce between true and found differences
	'RectOrient' : 0,  #degrees ideal from horizontal
	'RectAngleTol' : 5 #degrees from horizontal
	}

###### Low Target (Peg) ###################################################################
## Gear Peg target has two rectangular reflective tapes mounted vertically centered with
## the Peg in the middle.  They are both 2 inches wide by 5 inches tall and separated by
## 8.25 inches between their centerlines.


targetLow = dict(targetHigh)
targetLow['Rects'] =  [[2.0,5.0],[2.0,5.0]] #inches width x height for both rectangles

targetSought = 0 # High target camera = 0, Low target camera = 1

def selectTarget (targetSought = 0) :
	if targetSought == 0:
		camera = cameraHigh
		target = targetLow
	else :
		camera = cameraLow
		target = targetHigh
	return (camera,target)

camera, target = selectTarget(1)

aspectRatioTol = .1
areaRatio = 1.2 # tolerance for how close the contour matches a best fit rectanglar box
minBoxArea = 100 # minimum box size to consider	if ret==True:

def processFrame():

	ret, frame = camera.read()
	if ret==True:
		img2 = frame[:,:,1]  #green band
		ret,thresh = cv2.threshold(img2,200,255,0)
		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		boxes = []	#list of best fit boxes to contours
		boxCenters = [[]]  #centers of boxes
		for cnt in contours:
			rect = cv2.minAreaRect(cnt)  #minumum bounding rectangle of the contour
			box = cv2.boxPoints(rect) #best fit box (rotated) to the shape
			box = np.int0(box)
			# is it a big enough box?

			if cv2.contourArea(box) >= minBoxArea:
				if (cv2.contourArea(cnt) != 0): 
					# does it look like a rectangle?
					if (cv2.contourArea(box)/cv2.contourArea(cnt)) <= areaRatio: 
						# does it have the right orientation? 
						centerX, centerY = rect[0]
						width, height = rect[1]
						if height > width: 
							h = width
							width = height
							height = h
						if (height > 0):
							# First box
							targetWidth, targetHeight = target['Rects'][0]
							targetAspectRatio = targetWidth/targetHeight
							rectAspectRatio = width/height
							errorAspect = (rectAspectRatio-targetAspectRatio)/targetAspectRatio
							if (abs(errorAspect) <= aspectRatioTol):
								boxes.append(box) #collect the boxes for later processing
								if args.debug==True: cv2.drawContours(frame,[box], 0, 255, 3)
							# Second box								
							targetWidth, targetHeight = target['Rects'][1]
							targetAspectRatio = targetWidth/targetHeight
							rectAspectRatio = width/height
							errorAspect = (rectAspectRatio-targetAspectRatio)/targetAspectRatio
							if (abs(errorAspect) <= aspectRatioTol):
								boxes.append(box) #collect the boxes for later processing
								if args.debug==True: cv2.drawContours(frame,[box], 0, (0,0,255), 3)
	return frame

while(camera.isOpened()):
	start = time.time()
	frame = processFrame()

	if args.debug==True:
		cv2.imshow('Result',frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
		
	seconds = time.time() - start
	print "FPS: {0}".format(1/seconds)
	



# Release everything if job is finished
camera.release()
#out.release()
cv2.destroyAllWindows()