#!/usr/bin/env python

"""
Brooks Kindle
Kyle Finch
Cs 483 - Intro to robotics lab 6
"""

import time, sys
import rospy
import roslib; roslib.load_manifest('ardrone_tutorials')

import cv2
import copy
import numpy as np
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from diagnostic_msgs.msg import DiagnosticArray

from drone_controller import BasicDroneController
from PySide import QtCore, QtGui

from cv_bridge import CvBridge, CvBridgeError
import cv

class TagProcessing():
	def __init__(self):
		"""processes an image and then uses a callback to pass to another
		object"""
		print 'init in TagProcessing'
		#rospy.init_node("processor")
		self.points = None
		self.bridge = CvBridge()

	def getTags(self):
		"""Subscribes to the ARDrone front camera and returns a tuple of the
		points of the image. (screenx, screeny, objx, objy)"""
		self.points = (0,0,0,0)
		imgmsg = rospy.wait_for_message("/ardrone/front/image_raw", Image, timeout=None)
		self.points = self.ReceiveImage(imgmsg)
		return self.points

	def ReceiveImage(self, image):
		"""callback function -- gets called every time a new image comes in to /camera/rgb/image_raw"""
		#self.video.unregister()
		print "received image"
		cvimg = self.bridge.imgmsg_to_cv2(image, "bgr8") #convert ros image to cv image

		#####BEGIN MODIFYING THE IMAGE
		hsv = self.ConvertToHSV(cvimg) #optain hsv image
		thresh_lo = np.array([0, 150, 60])	 #low  threshold for red
		thresh_hi = np.array([255, 255, 180])	 #high threshold for red (doesn't really exist, we want LOTS of red)
		hsv = cv2.inRange(hsv, thresh_lo, thresh_hi) #convert to black && white
		#####END MODIFYING THE IMAGE

		#####BEGIN FINDING THE MIDPOINT
		rows, cols = hsv.shape #get dimensions of image
		x = 0
		y = 0
		for i in range(rows):
			for j in range(cols):
				if hsv[i][j].all() == 0:
					continue #all black pixel, not part of our image
				x = (x + j) / 2 #running average of x value
				y = (y + i) / 2 #running average of y value
#		print "calling callback function, and then unsubscribing"
#		self.callback(rows, cols, x, y)
#		self.video.unregister()
		#set the image points and unsubscribe
		#print "unregistering from subscription"
		return (cols/2, rows/2, x, y)

		#print "Midpoint found at ({}, {})".format(x, y)
		#####END FINDING THE MIDPOINT

		#show the modified image in a separate window
		#cv2.imshow("after", hsv)
		#cv2.waitKey(5)

	def ConvertToHSV(self, bgr_img):
		"""converts a cv image in BGR to HSV and returns the new image"""
		return cv2.cvtColor(bgr_img, cv.CV_BGR2HSV) #why do we have to import cv when we have cv2? Shouldn't that be depricated?
#end ImageProcessing

###########################################################################################
###########################################################################################
####################DO NOT GO ABOVE THIS LINE##############################################
#############################EVER##########################################################
###########################################################################################
###########################################################################################
def isAligned(screenx, screeny, objx, objy, offset):
	return objx >= screenx - offset and objx <= screenx + offset and objy >= screeny - offset and objy <= screeny + offset 
#end isAligned

def alignDrone(screenx, screeny, objx, objy, offset):
	"""
	Performs a small adjustment in the drone's positioning so that it better
	faces our target tag

	Parameters are as follows:
		screenx		-	number of pixels in the screen in the x direction
		screeny		-	number of pixels in the screen in the y direction
		objx		-	current x location of our tag
		objy		-	current y location of our tag
		offset		-	amount of pixel leeway to stay within
	"""
	global lastTagDirection
	#handle x direction adjustments
	if objx < screenx - offset:
		print "moving drone left"
		lastTagDirection = "left"
		p.publish(rotateCounterClockwise)
		#p.publish(balanceCounterClockwise)
	elif objx > screenx + offset:
		print "moving drone right"
		p.publish(rotateClockwise)
		lastTagDirection= "right"
		#p.publish(balanceClockwise)
	else:
		print "x direction aligned"
		#p.publish(stop)

	#rospy.sleep(.25)
	#handle y direction adjustments
	if objy > screeny + offset:
		print "moving drone down"
		p.publish(down)
		#p.publish(up)
	elif objy < screeny - offset:
		print "moving drone up"
		p.publish(up)
	else:
		print "y direction aligned"
		#p.publish(stop)

	#rospy.sleep(1.0)
	#p.publish(stop)
#end alignDrone

def onNavdataCapture(screenx, screeny, objx, objy, dist):
	"""
	Flies the ARDrone towards the tag.
	Parameters are as follows:
		screenx		-	number of pixels in the screen in the x direction
		screeny		-	number of pixels in the screen in the y direction
		objx		-	current x location of our tag
		objy		-	current y location of our tag
		dist		-	distance (in cm) away from our tag
	Return values are as follows:
		True		-	drone is within range of the tag
		False		-	drone is not within range of the tag
	"""
	global lastTagDirection
	done = False
	print "tag_x={} tag_y={} dist={}".format(objx, objy, dist)
	offset = 250

	if dist <= 100: #within range of tag
		p.publish(stop) #prevent movement
		done = True
		print "TAG WITHIN RANGE!@@@@@" * 3
	#end if
	elif objx or objy: #tag is in sight
		#p.publish(stop) #prevent movement
		if not isAligned(screenx, screeny, objx, objy, offset):
			alignDrone(screenx, screeny, objx, objy, offset) #align drone with center of object
		else: #we are aligned
			print "drone aligned"
			#p.publish(forward) #go forward a little bit
			for i in range(10):
				p.publish(forward)
				rospy.sleep(.1)
				#rospy.sleep(.25)
			#rospy.sleep(1.0)
	#end elif
	else:
		if lastTagDirection == None: #we've never found the tags before
			print 'object not found, rotating'
			p.publish(search)
		elif lastTagDirection == "left": #tag was left of the drone
			print 'object not found, rotating left'
			p.publish(searchLeft)
		else: #tag was right of the drone
			print 'object not found, rotating right'
			p.publish(searchRight)
		#p.publish(balanceClockwise)
		#rospy.sleep(1.0)
		#p.publish(stop)
	#end else	
	return done
#end onNavdataCapture

def keyPressEvent(event):
	key = event.key()

	# If we have constructed the drone controller and the key is not generated from an auto-repeating key
	if controller is not None and not event.isAutoRepeat():
		# Handle the important cases first!
		if key == KeyMapping.Emergency:
			print "SOS"
			controller.SendEmergency()

lastTagDirection = None
if __name__ == "__main__":
	#create our subscribers
	ptakeoff = rospy.Publisher('/ardrone/takeoff', Empty)
	pland = rospy.Publisher('/ardrone/land', Empty)
	p = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	#create our generic control messages
	forward = Twist(Vector3(3.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
	rotateClockwise = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.5))
	balanceClockwise = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0, 0.1))
	rotateCounterClockwise = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5))
	balanceCounterClockwise = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0, -0.1))
	search = rotateClockwise
	searchLeft = rotateCounterClockwise
	searchRight = rotateClockwise
	stop = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

	left = Twist(Vector3(0.0, 0.5, 0.0), Vector3(0.0, 0.0, 0.0))
	right = Twist(Vector3(0.0, -0.5, 0.0), Vector3(0.0, 0.0, 0.0))
	up = Twist(Vector3(0.0, 0.0, 0.5), Vector3(0.0, 0.0, 0.0))
	down = Twist(Vector3(0.0, 0.0, -0.5), Vector3(0.0, 0.0, 0.0))

	print "about to init our node"
	#now INIT OUR NODE
	rospy.init_node('ardrone')
	rospy.sleep(2)
	#should make an app with the controller
	#app = QtGui.QApplication(sys.argv)
	#controller = BasicDroneController()
	
	print "finished initting our node, about to execute the app"
	#status = app.exec_()
	print "done executing"

	#now do cool stuff
	try:
		if len(sys.argv) == 2 and sys.argv[1] == 'stop': #handle stopping
			p.publish(stop)
			pland.publish(Empty())
			sys.exit()

		ptakeoff.publish(Empty())
		rospy.sleep(4)
		p.publish(stop)
		#imageProcessing = TagProcessing(onImageCapture)
		done = False
		while not done:
			#introduce a small delay before getting the next reading
			#rospy.sleep(.10)
			#print "getting navdata from ardrone..."
			navmsg = rospy.wait_for_message("/ardrone/navdata", Navdata, timeout=None)
			#extract tag values from the navmsg
			#tag positioning is from top left, so the top left most pixel has
			#coordinate 0,0, while the bottom right has 999,999
			###################################################################
			#xc, yc, and dist are all tuples. If the drone is unable to see the
			#tag, then the tuples will all be empty. Otherwise, the information
			#will be in the 0th element of the tuple
			xc = navmsg.tags_xc #x position of the tag
			if len(xc) == 0: #object not found in the x position
				xc = 0
			else:
				xc = xc[0]
			yc = navmsg.tags_yc #y position of the tag 
			if len(yc) == 0: #object not found in the y position
				yc = 0
			else:
				yc = yc[0]
			dist = navmsg.tags_distance #distance from the tag (in cm)
			if len(dist) == 0: #tag not found
				dist = 9999 #some arbitrary high value
			else:
				dist = dist[0]
			done = onNavdataCapture(500, 500, xc, yc, dist)
		pland.publish(Empty())
	except KeyboardInterrupt: 
		print "Keyboard interrupt, shutting down"
		p.publish(stop)
		pland.publish(Empty())
