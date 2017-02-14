#!C:\Program Files\Anaconda2\python.exe
#
# Vision Processing Code for 2017 FRC
# FRC Team 3140 Farragut Flaship, Farragut High School, Knoxville Tennessee
#
# Python code intended to run on robot coprocessor (Kangaroo in our case)
# with 2 cameras.  A "High" target camera for the Boiler vision target and
# a "Low" target camera for the Gear Peg vision target.
#


from __future__ import print_function
import cv2
import numpy as np
import math
import time
import argparse
import socket
import re #used to check ip regex
import sys #
from pdb import set_trace as br

# Routines to parse command line arguments

parser = argparse.ArgumentParser(description="Finds 2017 Vision Targets")
parser.add_argument('--file', type=str, action='store', default=0, help='Video Filename instead of camera')
parser.add_argument('--thresh', default=False, action='store_const', const=True, help='Display Threshimg')
parser.add_argument('--debug', default=False, action='store_const', const=True, help='Debug Mode')
args=parser.parse_args()

# Define an error printing function for error reporting to terminal STD error IO stream

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

# Functions to handle UDP/IP communications

def initUdp(udp_ip,udp_port):
	global UDP_IP
	global UDP_PORT
	#set ip address
	try:
		assert type(udp_ip)==str
		if not re.match('\\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\\b',udp_ip):
			raise 'n cookie'
		UDP_IP = udp_ip
	except:
		eprint('Error: Provided udp_ip is not a valid ip')
		sys.exit()
	#set port
	try:
		assert type(udp_port)==int and udp_port in xrange(1,49151) #xrange is more memory efficient than range for large ranges
		UDP_PORT = udp_port
	except:
		eprint('Error: Provided port is invalid')
		sys.exit()
	#define socket
	UDP_SOCK = socket.socket(socket.AF_INET, # Internet
							 socket.SOCK_DGRAM) # UDP
	return UDP_SOCK

def udpSend(message,sock):
	sock.sendto(message, (UDP_IP, UDP_PORT))
	if args.debug:
		print('Sent:'+message)

def initCamera(id = 0):
	camera = cv2.VideoCapture(id)
	
	# Now we can initialize the camera capture object with the cv2.VideoCapture class.
	# All it needs is the index to a camera port.

	#camera.set(cv2.CV_CAP_PROP_FRAME_WIDTH, 640)
	#camera.set(cv2.CV_CAP_PROP_FRAME_HEIGHT, 480)

	if (id==0) : 			# max resolution for boiler target								
		xSize = 1280
		ySize = 720
	else:					# decent resolution for gear target
 		xSize = 640
		ySize = 480
											
	camera.set(cv2.CAP_PROP_FRAME_WIDTH, xSize)
	camera.set(cv2.CAP_PROP_FRAME_HEIGHT, ySize)
	camera.set(cv2.CAP_PROP_BRIGHTNESS, 220) 
	camera.set(cv2.CAP_PROP_CONTRAST, 10) 
	camera.set(cv2.CAP_PROP_EXPOSURE,-11) 

	resX = fovX / xSize		# degrees/pixel
	resY = fovY / ySize		# degrees/pixel

	return (resX,resY,camera)

############## Parameter Initialization #############################/33333#####################
filename = args.file
font = cv2.FONT_HERSHEY_SIMPLEX
aspectRatioTol = .4	# tolerance on the degree of fit in width/height aspects to expected
areaRatio = 1.2 	# tolerance for how close the contour matches a best fit rectanglar box
minBoxArea = 100 	# minimum box size to consider	if ret==True:
targetSought = 0 	# High target camera = 0, Low target camera = 1 
sepPixelTol = 15 	# pixel error tolerance on expected separations of targets in the frame
start_time=time.time() #for diagnostics
runtime = start_time
fpsMin = 10000000
fpsMax = -1
fpsCount = 0
fpsSum = 0
fovX = math.radians(62.39)			#  Horizontal FOV estimated for MS Lifecam 3000 HD
fovY = math.radians(34.3)			# Vertical FOV for MS Lifecam 3000 HD
#fovX = math.radians(62.8)			#  Horizontal FOV estimated for MS Lifecam 3000 HD
#fovY = math.radians(37.9)			# Vertical FOV for MS Lifecam 3000 HD
cameraAngle = math.radians(0.0)		# degrees inclination
imageBinaryThresh = 100				# Threshold to binarize the image data
send_sock=initUdp('10.31.40.42',5803) # initializes UDP socket to send on (RobioRio static IP)
##############################################################################################
#
# Target Definitions - for a High vision target (boiler) and Low target (Gear placement)
# Defined as attributes of rectangles and their expected interdependices with 
# each other and the background
#
###### High Target (Boiler) ##################################################################
#
# Boiler target has two rectangular reflective tapes mounted horizontally around the
# funnel.  They are parallel separated by 2 inches.  The top tape is 4 inches tall and
# the bottom tape is 2 inches tall.  The diameter of the funnel they circle is 15 inches.
# The tape won't reflect off axis so it will likely appear less than the full 15 inches.

targetHigh = {
	'NumRects' : 2,
	'Rects' : [[14.0,4.0],[14.0,2.0]], #inches width x height for both rectangles
	'RectSep' : [0.0,7.0], #inches width, height in separation between rectangle centers
	'RectIntensity' : [True,True], #each rectangle should be brighter than surrounding
	'RectSepTol' : 0.25, #inches tolernce between true and found differences
	'RectOrient' : 0,  #degrees ideal from horizontal
	'RectAngleTol' : 5 #degrees from horizontal
	}

###### Low Target (Peg) ######################################################################
#
# Gear Peg target has two rectangular reflective tapes mounted vertically centered with
# the Peg in the middle.  They are both 2 inches wide by 5 inches tall and separated by
# 8.25 inches between their centerlines.

targetLow = dict(targetHigh)
targetLow['Rects'] =  [[2.0,5.0],[2.0,5.0]] #inches width x height for both rectangles


if args.file:
	camera = cv2.VideoCapture(filename)
	cameraHigh = camera	# cameras are not used when reading from a saved test video
	cameraLow = camera	# cameras are not used when reading from a saved test video
	xSize = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
	ySize = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
	resX = fovX / xSize
	resY = fovY / ySize
	resXHigh = resX
	resXLow = resX
	resYHigh = resY
	resYLow = resY
else:
	resXHigh, resYHigh, cameraHigh = initCamera(0)
	resXLow, resYLow, cameraLow = initCamera(1)


def selectTarget (targetSought = 0) :
	if targetSought == 0:
		camera = cameraHigh
		target = targetHigh
		xSize = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
		ySize = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
		resX = resXHigh
		resY = resYHigh
	else :
		camera = cameraLow
		target = targetLow
		xSize = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
		ySize = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
		resX = resXLow
		resY = resYLow
	return (resX,resY,xSize,ySize,camera,target)

resX, resY, xSize, ySize, camera, target = selectTarget(0)

def processFrame():			# This function does all of the image processing on a single frame

	boxes = []	#list of best fit boxes to contours
	boxCenters = [[]]  #centers of boxes
	ret, frame = camera.read()
	thresh = 0
	found = False
	segments1 = []		# found segment array for rectangle 1
	segments2 = []		# found segment array for rectangle 2
	aimPoint = []		# empty until found
	slantRange = -1.0	# negative means not set
	bearing = 1.e6		# nonsense until set

	if ret==True:
		img2 = frame[:,:,1] # green band used only as we are using green LED illuminators
		ret,thresh = cv2.threshold(img2,imageBinaryThresh,255,cv2.THRESH_BINARY)	# get a binary image of only the brightest areas
		img2 = thresh.copy()
		im2, contours, hierarchy = cv2.findContours(img2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		for cnt in contours:
			rect = cv2.minAreaRect(cnt)  #minumum bounding rectangle of the contour
			box = cv2.boxPoints(rect) #best fit box (rotated) to the shape
			
			centerX, centerY = rect[0]

			topLeft = box[0]	# make certain always has a value
			topRight = box[0]
			botLeft = box[0]
			botRight = box[0]
			for i in range(1,4):
				if (box[i][0] < centerX) and (box[i][1] < centerY): topLeft = box[i]
				if (box[i][0] < centerX) and (box[i][1] > centerY): botLeft = box[i]
				if (box[i][0] > centerX) and (box[i][1] < centerY): topRight = box[i]
				if (box[i][0] > centerX) and (box[i][1] > centerY): botRight = box[i]

			edgeTop = [(topLeft[0]+topRight[0])/2, (topLeft[1]+topRight[1])/2]
			edgeBot = [(botLeft[0]+botRight[0])/2, (botLeft[1]+botRight[1])/2]
			edgeLeft = [(topLeft[0]+botLeft[0])/2, (topLeft[1]+botLeft[1])/2]
			edgeRight = [(topRight[0]+botRight[0])/2, (topRight[1]+botRight[1])/2]

			width = edgeRight[0] - edgeLeft[0]
			height = edgeBot[1] - edgeTop[1]

			rect = ((centerX,centerY),(width,height), 0.)
			box = np.int0(box)
			# is it a big enough box?

			if cv2.contourArea(box) >= minBoxArea:
				if (cv2.contourArea(cnt) != 0): 
					# does it look like a rectangle?
					if (cv2.contourArea(box)/cv2.contourArea(cnt)) <= areaRatio: 
						# does it have the right orientation? 
						centerX, centerY = rect[0]
						width, height = rect[1]
						if (height > 0):
							# First box
							targetWidth, targetHeight = target['Rects'][0]
							targetAspectRatio = targetWidth/targetHeight
							rectAspectRatio = width/height
							errorAspect = (rectAspectRatio-targetAspectRatio)/targetAspectRatio
							if (abs(errorAspect) <= aspectRatioTol):
								segments1.append(rect) #collect the boxes for later processing
								if args.debug==True: 
									cv2.drawContours(frame,[box], 0, 255, 2)
							# Second box								
							targetWidth, targetHeight = target['Rects'][1]
							targetAspectRatio = targetWidth/targetHeight
							rectAspectRatio = width/height
							errorAspect = (rectAspectRatio-targetAspectRatio)/targetAspectRatio
							if (abs(errorAspect) <= aspectRatioTol):
								segments2.append(rect) #collect the boxes for later processing
								if args.debug==True: 
									cv2.drawContours(frame,[box], 0, (0,0,255), 2)


	if (len(segments1) > 0) and (len(segments2) > 0):			# any candidate pairs?
		# let's see if the ratios we found are consistent between all the segments
		# at a given range, all target segment ratios should match all segment ratios relative to each other

		target1Width, target1Height = target['Rects'][0]
		target2Width, target2Height = target['Rects'][1]

		targetWidthRatio = target1Width / target2Width			
		targetHeightRatio = target1Height / target2Height

		for rect1 in segments1:
			width1, height1 = rect1[1]

			for rect2 in segments2:	
				width2, height2 = rect2[1]
				heightRatio = height1 / height2
				heightErrorAspect = (heightRatio - targetHeightRatio) / targetHeightRatio

				if (abs(heightErrorAspect) <= aspectRatioTol):
					widthRatio = width1 / width2
					widthErrorAspect = (widthRatio - targetWidthRatio) / targetWidthRatio

					if (abs(widthErrorAspect) <= aspectRatioTol):
						# each segment appears the right size relative to each other, how about the expected 
						# separation relative to each other?  Does that match as well on the segments?

						targetSepX, targetSepY = target['RectSep']
						resApparentX = target1Width / width1			# if top segment is true, this estimates inches/pixel
						resApparentY = target1Height / height1			# if top segment is true, this estimates inches/pixel
						targetSepXPixels = targetSepX / resApparentX
						targetSepYPixels = targetSepY / resApparentY
						center1X, center1Y = rect1[0]
						center2X, center2Y = rect2[0]
						segmentSepX = center2X - center1X				# row, col coordinate system with top left the origin
						segmentSepY = center2Y - center1Y
						sepXError = (segmentSepX - targetSepXPixels)
						sepYError = (segmentSepY - targetSepYPixels)

						if ((abs(sepXError) <= sepPixelTol) and (abs(sepYError) <= sepPixelTol)):
							found = True
							minX = 1.e6
							minY = 1.e6
							maxX = -1
							maxY = -1

							box = cv2.boxPoints(rect1)
							for i in range(0,4):
								if (box[i][0] < minX): minX = box[i][0]
								if (box[i][0] > maxX): maxX = box[i][0]
								if (box[i][1] < minY): minY = box[i][1]
								if (box[i][1] > maxY): maxY = box[i][1]

							box = cv2.boxPoints(rect2)
							for i in range(0,4):
								if (box[i][0] < minX): minX = box[i][0]
								if (box[i][0] > maxX): maxX = box[i][0]
								if (box[i][1] < minY): minY = box[i][1]
								if (box[i][1] > maxY): maxY = box[i][1]

							foundBox = [[minX,minY],
										[maxX,minY],
										[maxX,maxY],
										[minX,maxY]]	
							br()
							targetTotalHeight = (target1Height/2.0) + (target2Height/2.0) + targetSepY 			# inches
							slantRange = (targetTotalHeight/12.0 * ySize) / (2.0 * (maxY-minY) * math.tan(fovY))
							aimPoint = [minX + (maxX-minX)/2.0, minY + (maxY-minY)/2.0]
							bearing = (aimPoint[0] - xSize/2.0) * math.degrees(resX)
							foundBox = np.array(foundBox, dtype=np.int32)

							if args.debug==True: 
								cv2.drawContours(frame,[foundBox], 0, (255,255,0), 2)
								apX = np.int0(aimPoint[0])	
								apY = np.int0(aimPoint[1])
								aimLineStart = (apX-10,apY)
								aimLineStop  = (apX+10,apY)
								cv2.line(frame,aimLineStart,aimLineStop,(0,255,255),3)
								aimLineStart = (apX,apY-10)
								aimLineStop  = (apX,apY+10)							
								cv2.line(frame,aimLineStart,aimLineStop,(0,255,255),3)

	return (ret, thresh, frame, found, aimPoint, slantRange, bearing)

while(camera.isOpened()):								# Main Processing Loop
	runtimeLast = runtime
	ret, thresh, frame, found, aimPoint, slantRange, bearing = processFrame()

	if ret:		
		runtime=time.time()-start_time

#		udpSend(str(runtime)+',12,34,Last',send_sock)
		if (args.debug):
			fps = 1.0/(runtime - runtimeLast)
			fps = np.int0(fps)
			fpsCount = fpsCount + 1
			fpsSum = fpsSum + fps
			fpsAvg = fpsSum / fpsCount
			if (fpsCount > 1) and (fps < fpsMin): fpsMin = fps  #discard first time through
			if fps > fpsMax: fpsMax = fps

			if (args.thresh):
				show = thresh
			else:
				show = frame

			cv2.putText(show,'FPS: '+str(fps),(10,30),font,0.5,(255,255,255),1)
			cv2.putText(show,'FPS Min: '+str(fpsMin),(10,50),font,0.5,(255,255,255),1)
			cv2.putText(show,'FPS Max: '+str(fpsMax),(10,70),font,0.5,(255,255,255),1)
			cv2.putText(show,'FPS Avg: '+str(fpsAvg),(10,90),font,0.5,(255,255,255),1)	
			if found:
				slantRangeStr = 'Slant Range (ft): '+"%0.2f" % (slantRange)
				cv2.putText(show,slantRangeStr,(10,110),font,0.5,(255,255,255),1)
				bearingStr = 'Bearing (deg): '+"%0.2f" % (bearing)
				cv2.putText(show,bearingStr,(10,130),font,0.5,(255,255,255),1)				

			cv2.imshow('Result',show)

		if (cv2.waitKey(1) & 0xFF == ord('q')):
			break
	else: break


		


# Release everything if job is finished
camera.release()
#out.release()
cv2.destroyAllWindows()
