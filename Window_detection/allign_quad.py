#!/usr/bin/env python

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from window_detection import *
# step size(dt) = 0.5 @ 10 hz publish rate = 1 meter dist travel
class allign_quad :
	def __init__(self):
		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		self.pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
		
		self.inFlight = False
		self.pose = Pose()
		self.twist = Twist()
		self.bias_x = 0
		self.bias_y = 0
		self.bias_z = 0
		self.prev_error = 0

	def calculate_bias(self):
		self.bias_x = self.pose.position.x
		self.bias_y = self.pose.position.y
		self.bias_z = self.pose.position.z	

	def takeoff(self):
		self.takeoff_pub.publish()
		self.inFlight = True

	def odom_callback(self, data):
		self.pose = data.pose.pose
		# print "pose ", self.pose
		self.Twist = data.twist.twist
		# print "twist ", self.Twist

	def some_fx(self):
		
		velocity_msg = Twist()


if __name__ == '__main__':
	pid = pid_controller()
	pid.straight_line()
	# sin_xy()
	# rhombus_traj()
	# sawtooth_traj()