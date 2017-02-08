import cv2
import numpy as np
import time
import argparse

parser = argparse.ArgumentParser(description="Finds 2017 Vision Targets")
parser.add_argument('--debug', default=False, action='store_const', const=True, help='Debug Mode')
args=parser.parse_args()

from pdb import set_trace as br

# Camera 0 is the High target camera
# Camera 1 is the Low target camera

cameraHigh = cv2.VideoCapture(0)
cameraLow = cv2.VideoCapture(1)
  
# Now we can initialize the camera capture object with the cv2.VideoCapture class.
# All it needs is the index to a camera port.

#camera.set(cv2.CV_CAP_PROP_FRAME_WIDTH, 640)
#camera.set(cv2.CV_CAP_PROP_FRAME_HEIGHT, 480)

cameraHigh.set(3, 640)
cameraHigh.set(4, 480)
cameraLow.set(3, 640)
cameraLow.set(4, 480)

# Target Definitions - for a High vision target (boiler) and Low target (Gear placement)
# Defined as attributes of rectangles and their expected interdependices with 
# each other and the background

###### High Target (Boiler)################################################################
## Boiler target has two rectangular reflective tapes mounted horizontally around the
## funnel.  They are parallel separated by 2 inches.  The top tape is 4 inches tall and
## the bottom tape is 2 inches tall.  The diameter of the funnel they circle is 15 inches.
## The tape won't reflect off axis so it will likely appear less than the full 15 inches.

targetHighNumRects = 2
targetHighRects = [[16.0,4.0],[16.0,2.0]] #inches width x height for both rectangles
targetHighRectSep = [0.0,5.0] #inches width, height in separation between rectangles
targetHighRectIntensity = [True,True] #each rectangle should be brighter than surrounding
targetHighRectSepTol = 0.25 #inches tolernce between true and found differences
targetHighRectOrient = 0 #degrees ideal from horizontal
targetHighRectAngleTol = 5 #degrees from horizontal

###### Low Target (Peg) ###################################################################
## Gear Peg target has two rectangular reflective tapes mounted vertically centered with
## the Peg in the middle.  They are both 2 inches wide by 5 inches tall and separated by
## 8.25 inches between their centerlines.

targetLowNumRects = 2
targetLowRects = [[2.0,5.0],[2.0,5.0]] #inches width x height for both rectangles
targetLowRectSep = [8.25,0.0] #inches width, height in separation between rectangles
targetLowRectIntensity = [True,True] #each rectangle should be brighter than surrounding
targetLowRectSepTol = 0.25 #inches tolernce between true and found differences
targetLowRectOrient = -90 #degrees ideal from horizontal
targetLowRectAngleTol = 5 #degrees from vertical

targetSought = 0 # High target camera = 0, Low target camera = 1

if targetSought == 0 :
	camera = cameraHigh
	targetNumRects = targetHighNumRects
	targetRects = targetHighRects
	targetRectSep =targetHighRectSep
	targetRectIntensity = targetHighRectIntensity
	targetRectSepTol = targetHighRectSepTol
	targetRectOrient = targetHighRectOrient
	targetRectAngleTol = targetHighRectAngleTol
else :
	camera = cameraLow
	targetNumRects = targetLowNumRects
	targetRects = targetLowRects
	targetRectSep =targetLowRectSep
	targetRectIntensity = targetLowRectIntensity
	targetRectSepTol = targetLowRectSepTol
	targetRectOrient = targetLowRectOrient
	targetRectAngleTol = targetLowRectAngleTol


# Define the codec and create VideoWriter object
#fourcc = cv2.cv.CV_FOURCC(*'DVIX')
#out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

aspectRatioTol = 0.1
areaRatio = 1.2 # tolerance for how close the contour matches a best fit rectanglar box
minBoxArea = 100 # minimum box size to consider

while(camera.isOpened()):
	start = time.time()
	ret, frame = camera.read()
	if ret==True:
		img2 = frame[:,:,1]  #green band
		ret,thresh = cv2.threshold(img2,200,255,0)
		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# just look at the top largest contours (sorted)

#		contours = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
		
# look for best fit boxes to contours (contour looks like a rectangle)

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
							targetWidth, targetHeight = targetRects[0]
							targetAspectRatio = targetWidth/targetHeight
							rectAspectRatio = width/height
							errorAspect = (rectAspectRatio-targetAspectRatio)/targetAspectRatio
#							print "rectAspect:", rectAspectRatio
#							print "targetAspect:", targetAspectRatio
#							print "errorAspect:", errorAspect
							if (abs(errorAspect) <= aspectRatioTol):
								
#								print centerX, centerY
#								print width, height
#								print angle
								boxes.append(box) #collect the boxes for later processing
								cv2.drawContours(frame,[box], 0, 255, 3)								
							targetWidth, targetHeight = targetRects[1]
							targetAspectRatio = targetWidth/targetHeight
							rectAspectRatio = width/height
							errorAspect = (rectAspectRatio-targetAspectRatio)/targetAspectRatio

							if (abs(errorAspect) <= aspectRatioTol):
#								print "rectAspect:", rectAspectRatio
#								print "targetAspect:", targetAspectRatio
#								print "errorAspect:", errorAspect								
#								print centerX, centerY
#								print width, height
#								print angle
								boxes.append(box) #collect the boxes for later processing
								cv2.drawContours(frame,[box], 0, (0,0,255), 3)
#							boxMoments = cv2.moments(box)
#							cX = int(boxMoments["m10"] / boxMoments["m00"])
#							cY = int(boxMoments["m01"] / boxMoments["m00"])
#							boxCenters.append([cX,cY]) 

#								cv2.circle(frame,(centerX,centerY), 3, (0,255,0), -1)
#				print cv2.contourArea(box)/cv2.contourArea(cnt)
#			cv2.drawContours(frame, [cnt], 0, (0,255,0), 3)

		cv2.imshow('Result',frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
			
		seconds = time.time() - start
		print "FPS: {0}".format(1/seconds)
	
#       out.write(frame)
	else:#
		break

# Release everything if job is finished
camera.release()
#out.release()
cv2.destroyAllWindows()

