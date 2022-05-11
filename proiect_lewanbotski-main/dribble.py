#!/usr/bin/env python3
import sys
import rospy
import cv2
from cv_bridge import CvBridge 

from numpy import array  
from numpy.linalg import norm 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import time

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Callback function called whenever
# x-y coordinate received

D = array([0,0])

stage		= 0

distance	= 0

roll		= 0
pitch		= 0
yaw			= 0

# Ball position on map
x_Ball		= 0	
y_Ball		= 0

# Robot position on map
x_Robot		= 0
y_Robot 	= 0
rx_Robot 	= 0
ry_Robot 	= 0
rz_Robot 	= 0

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

class Server:
	def __init__(self):
		self.norm 	= None
		self.ball_x 	= None		#Ball position on camera
		self.ball_y 	= None
		self.width  	= None

		self.roll		= None
		self.pitch		= None
		self.yaw		= None
	
	#receive ball's position on camera
	def drive_callback(self,data):

		self.ball_x 	= data.x
		self.ball_y 	= data.y
		self.width  	= data.z
		
		self.compute()

	#receive distance to ball
	def distance_callback(self,msg):
		global distance
		distance = msg.ranges[0]
		self.compute()

	#receive robot's angle and position
	def rotation_callback(self,msg):
		global roll,pitch,yaw
		global x_Robot,y_Robot,rx_Robot,ry_Robot,rz_Robot

		orientation_q		= msg.pose.pose.orientation
		orientation_list	= [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
		(roll,pitch,yaw)	= euler_from_quaternion(orientation_list)

		x_Robot = msg.pose.pose.position.x
		y_Robot = msg.pose.pose.position.y

		rx_Robot = msg.pose.pose.orientation.x
		ry_Robot = msg.pose.pose.orientation.x
		rz_Robot = msg.pose.pose.orientation.x

		self.compute()

	def compute(self):
		global stage
		global x_Ball,y_Ball
		global D
		global client

		# Create Twist() instance
		vel = Twist()
		
		# find ball
		if stage==0:
			if self.ball_x < 0 and self.ball_y < 0:
				vel.angular.z = 0.8
			else:
				# Determine center-x, normalized deviation from center
				mid_x  	= int(self.width/2)
				delta_x	= self.ball_x - mid_x
				self.norm_x 	= delta_x/self.width

				if self.norm_x > 0.02:
					vel.angular.z = -0.2
				elif self.norm_x < -0.02:
					vel.angular.z = 0.2
				if abs(self.norm_x) < 0.02:		
					vel.angular.z = 0
					stage=1
					
			# publish vel on the publisher
			pub_vel.publish(vel)
		
		# calculate coordinates of ball
		if stage==1:

			print("stage 1")
			if distance > 0 and distance < 4:
				x_Ball 	= (distance+0.05) * math.cos(yaw)
				y_Ball  = (distance+0.05) * math.sin(yaw)
				stage=2

		# go behind ball
		if stage==2:

			A = array([2,0])
			B = array([x_Ball,y_Ball])

			#distance left behind ball
			e=0.4

			#coordinates of robot behind ball
			D = B - e*(A-B)/norm(A-B)

			#set robot's goal 
			client.wait_for_server()
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = D[0]
			goal.target_pose.pose.position.y = D[1]
			goal.target_pose.pose.orientation.x = 0
			goal.target_pose.pose.orientation.y = 0
			goal.target_pose.pose.orientation.z = 0
			goal.target_pose.pose.orientation.w = 1.0

			stage=3
			client.send_goal(goal)

		# wait for the robot to get behind the ball
		if stage==3:
			if x_Robot < D[0] +0.05 and x_Robot >D[0]-0.05:
				if y_Robot < D[1] +0.05 and y_Robot >D[1]-0.05:
					if rx_Robot < 0.05 and rx_Robot < 0.05 and rx_Robot < 0.05 :
						client.cancel_all_goals()
						stage=4

		# center on ball
		if stage==4:
			#if robot is near goal then go to next stage
			if(x_Robot>1.8):
				stage=5

			if self.ball_x < 0 and self.ball_y < 0:
				vel.angular.z = 0.6
			else:
				# Determine center-x, normalized deviation from center
				mid_x  	= int(self.width/2)
				delta_x	= self.ball_x - mid_x
				self.norm_x 	= delta_x/self.width

				if self.norm_x > 0.04:
					vel.angular.z = -0.2
				elif self.norm_x < -0.04:
					vel.angular.z = 0.2
				if abs(self.norm_x) < 0.04:
					vel.angular.z = 0
					vel.linear.x=0.15
					
			# publish vel on the publisher
			pub_vel.publish(vel)	

		# command the robot to return to origin
		if stage==5:

			client.wait_for_server()

			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose.position.x = 0
			goal.target_pose.pose.position.y = 0
			goal.target_pose.pose.orientation.x = 0
			goal.target_pose.pose.orientation.y = 0
			goal.target_pose.pose.orientation.z = 0
			goal.target_pose.pose.orientation.w = 1.0

			stage=6
			client.send_goal(goal)

		# wait for robot to get to origin
		if stage==6:
			if x_Robot < 0.05 and x_Robot >-0.05:
				if y_Robot <0.05 and y_Robot >-0.05:
					if rx_Robot < 0.05 and rx_Robot < 0.05 and rx_Robot < 0.05 :
						client.cancel_all_goals()
						stage=7

		# GOOOOAL !!! -hopefully
		if stage==7:
			print("Done! You can Ctrl+C this !")
			client.cancel_all_goals()
			stage=8



if __name__ == '__main__':
	global vel, pub_vel
	
	# create server instance
	server = Server()
		
	# intialize the node
	rospy.init_node('drive_wheel', anonymous=True)

	# subscribe to /ball_location topic to receive coordinates
	img_sub = rospy.Subscriber("/ball_location",Point, server.drive_callback)
	
	# publish to /cmd_vel topic the angular-z velocity change
	pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

	# subscribe to /scan topic to receive distance to ball
	dist_sub = rospy.Subscriber('/scan',LaserScan,server.distance_callback)
	
	# subscribe to /odom topic to receive angle and position
	quat_sub = rospy.Subscriber('/odom',Odometry,server.rotation_callback)




	rospy.spin()
	
	
	
	
