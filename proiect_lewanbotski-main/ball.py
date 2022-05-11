#!/usr/bin/env python3
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

def detectBall(frame):
	global counter, X, Y
	counter += 1

	# blue
	colorLower = ( 90, 70,100)
	colorUpper = (120,200,255)
	i1, i2 = 0, 5

	# Convert to HSV color-space and create the mask
	hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, colorLower, colorUpper)
	mask = cv2.erode(mask, None, iterations=i1)
	mask = cv2.dilate(mask, None, iterations=i2)
	
	# Find all contours after a series of erosion/dilation
	(cnts, _) = cv2.findContours(mask.copy(), \
        cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	# Initializing x & y lists every nth frame
	if counter%4 == 0:
		X, Y = [], []

	# For each contour, get area, perimeter, filter, and find centroid
	for (i,c) in enumerate(cnts):
		area = cv2.contourArea(c)
		perimeter = cv2.arcLength(c, True)
		if area > 1000 and perimeter > 100:
			print ("Contour #%d -- area: %.2f, perimeter: %.2f" \
				% (i + 1, area, perimeter))			
			c = max(cnts, key=cv2.contourArea)
			M = cv2.moments(c)
			(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			X.append(cX)
			Y.append(cY)

	# Average out each x & y location determined
	if X and Y:
		cX = int(sum(X)/len(X))
		cY = int(sum(Y)/len(Y))
		return cX, cY, mask
	else:
		return -100, -100, mask
def image_callback(data):
	global cX, cY, pub

	# convert to numpy -> RGB
	image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
	h , w = image.shape[:2]

	# image = imutils.resize(image, width=int(w*8))
	cX, cY, mask = detectBall(image)

	# Create Point instance and set x, y methods
	point = Point()
	point.x = cX
	point.y = cY
	point.z = w 	# width of the frame
	pub_point.publish(point)		# Publish point on the publisher


if __name__ == '__main__':
	global counter, X, Y, cX, cY, pub
	counter = 0
	X, Y = [], []

	# Initialize the node
	rospy.init_node('find_ball', anonymous=True)

	# Subscribe to raspicam_node and receive the image
	img_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,image_callback)
	bridge = CvBridge()
	
	# Publish x-y coordinates over ball_location topic
	pub_point = rospy.Publisher('/ball_location', Point, queue_size=5)
	rospy.spin()
