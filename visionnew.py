#!C:\Program Files\Anaconda2\python.exe
from __future__ import print_function
import cv2
import numpy as np
import time
import argparse
import socket
import re #used to check ip regex
import sys #
from pdb import set_trace as br

parser = argparse.ArgumentParser(description="Finds 2017 Vision Targets")
parser.add_argument('--file', type=str, action='store', default=0, help='Video Filename instead of camera')
parser.add_argument('--thresh', default=False, action='store_const', const=True, help='Display Threshimg')
parser.add_argument('--debug', default=False, action='store_const', const=True, help='Debug Mode')
args=parser.parse_args()

#define an error printing function for error reporting to terminal STD error IO stream
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

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

send_sock=initUdp('10.31.40.42',5803) # initializes UDP socket to send on (RobioRio static IP)

def initCamera(id = 0):
	camera = cv2.VideoCapture(id)
	
	# Now we can initialize the camera capture object with the cv2.VideoCapture class.
	# All it needs is the index to a camera port.

	#camera.set(cv2.CV_CAP_PROP_FRAME_WIDTH, 640)
	#camera.set(cv2.CV_CAP_PROP_FRAME_HEIGHT, 480)

#	camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
#	camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
	camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
	camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) 
	camera.set(cv2.CAP_PROP_BRIGHTNESS, 220) 
	camera.set(cv2.CAP_PROP_CONTRAST, 10) 
	camera.set(cv2.CAP_PROP_EXPOSURE,-11) 
	return camera

############## Parameter Initialization ##################################################
filename = args.file
font = cv2.FONT_HERSHEY_SIMPLEX
aspectRatioTol = .4
areaRatio = 1.2 # tolerance for how close the contour matches a best fit rectanglar box
minBoxArea = 100 # minimum box size to consider	if ret==True:
targetSought = 0 # High target camera = 0, Low target camera = 1 
start_time=time.time() #for diagnostics
runtime = start_time
fpsMin = 10000000
fpsMax = -1
fpsCount = 0
fpsSum = 0
#########################################################################################

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
	'Rects' : [[14.0,4.0],[14.0,2.0]], #inches width x height for both rectangles
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

def selectTarget (targetSought = 0) :
	if targetSought == 0:
		camera = cameraHigh
		target = targetHigh
	else :
		camera = cameraLow
		target = targetLow
	return (camera,target)

if args.file:
	camera = cv2.VideoCapture(filename)
	cameraHigh = camera	# cameras are not used when reading from a saved test video
	cameraLow = camera	# cameras are not used when reading from a saved test video
else:
	cameraHigh = initCamera(0)
	cameraLow = initCamera(1)

camera, target = selectTarget(0)

def processFrame():

	boxes = []	#list of best fit boxes to contours
	boxCenters = [[]]  #centers of boxes
	ret, frame = camera.read()
	thresh = 0

	if ret==True:
		img2 = frame[:,:,1]  #green band
		ret,thresh = cv2.threshold(img2,100,255,0)
		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		for cnt in contours:
			rect = cv2.minAreaRect(cnt)  #minumum bounding rectangle of the contour
			box = cv2.boxPoints(rect) #best fit box (rotated) to the shape
			box = np.int0(box)
			# is it a big enough box?

			segments1 = [[]]
			segments2 = [[]]
			if cv2.contourArea(box) >= minBoxArea:
				if (cv2.contourArea(cnt) != 0): 
					# does it look like a rectangle?
					if (cv2.contourArea(box)/cv2.contourArea(cnt)) <= areaRatio: 
						# does it have the right orientation? 
						centerX, centerY = rect[0]
						width, height = rect[1]
						if height > width: 	# make insensitive to 90 deg rotations due to minAreaRect results
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
		found = False
		# let's see if the ratios we found are consistent between all the segments
		# at a given range, all target segment ratios should match all segment ratios relative to each other
		target1Width, target1Height = target['Rects'][0]
		target2Width, target2Height = target['Rects'][1]
		targetWidthRatio = target1Width / target2Width			
		targetHeightRatio = target1Height / target2Height
		for rect1 in segments1:
			width1, height1 = segments1[1]
			for rect2 in segments2:	
				width2, height2 = segments2[1]
				heightRatio = height1 / height2
				heightErrorAspect = (heightRatio / targetHeightRatio) / targetHeightRatio
				if (abs(heightErrorAspect) <= aspectRatioTol):
					widthRatio = width1 / width2
					widthErrorAspect = (widthRatio / targetWidthRatio) / targetWidthRatio
					if (abs(widthErrorAspect) <- aspectRatioTol):
						# each segment appears the right size relative to each other, how about the expected 
						# separation relative to each other?  Does that match as well on the segments?
						targetSepX, targetSepY = target['RectSep']
						center1X, center1Y = segments1[0]
						center2X, center2Y = segments2[0]
						segmentSepX = center2X - center1X		# row, col coordinate system with top left the origin
						segmentSepY = cetner2Y - cetner2Y




	return ret, thresh, frame, boxCenters

while(camera.isOpened()):
	runtimeLast = runtime
	ret, thresh, frame, boxCenters = processFrame()

	if ret:		
		runtime=time.time()-start_time

		udpSend(str(runtime)+',12,34,Last',send_sock)
		if (args.debug):
			fps = 1.0/(runtime - runtimeLast)
			fps = np.int0(fps)
			fpsCount = fpsCount + 1
			fpsSum = fpsSum + fps
			fpsAvg = fpsSum / fpsCount
			if (fpsCount > 1) and (fps < fpsMin): fpsMin = fps  #discard first time through
			if fps > fpsMax: fpsMax = fps
			cv2.putText(frame,'FPS: '+str(fps),(10,30),font,0.5,(255,255,255),1)
			cv2.putText(frame,'FPS Min: '+str(fpsMin),(10,50),font,0.5,(255,255,255),1)
			cv2.putText(frame,'FPS Max: '+str(fpsMax),(10,70),font,0.5,(255,255,255),1)
			cv2.putText(frame,'FPS Avg: '+str(fpsAvg),(10,90),font,0.5,(255,255,255),1)	
			if args.thresh:
				cv2.imshow('Thresholded Image',thresh)		
			else:
				cv2.imshow('Result',frame)

		if (cv2.waitKey(1) & 0xFF == ord('q')):
			break
	else: break


		


# Release everything if job is finished
camera.release()
#out.release()
cv2.destroyAllWindows()
