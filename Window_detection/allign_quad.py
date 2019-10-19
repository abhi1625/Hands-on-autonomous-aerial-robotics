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
from GMM.test_data import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class video_stream:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_raw", Image, self.img_callback)
		self.image_data = None

	def img_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		self.image_data = cv_image

	def get_img(self):
		return self.image_data

# step size(dt) = 0.5 @ 10 hz publish rate = 1 meter dist travel
class allign_quad :
	def __init__(self):
		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		self.vel_pub = rospy.Publisher('/cmd_vel_setpoint', Pose, queue_size=10)
		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1 , latch=True)
		
		self.inFlight = False
		self.pose = Pose()
		self.twist = Twist()
		self.bias_x = 0
		self.bias_y = 0
		self.bias_z = 0
		self.prev_error = 0
		self.vs = video_stream()
		self.velocity_msg = Twist()
		self.image = self.vs.get_img()
		weights_path = './GMM/training_params/window_weights.npy'
		params_path = './GMM/training_params/gaussian_params.npy'
		self.n, self.K, self.weights, self.params = loadparamsGMM(weights_path, params_path)
		self.areas = []
		self.odom_list = []
		self.dt = 0.1
		self.Kp = [4.5, 4.5, 1.0,0,0,6]
		self.inFlight = False
		self.vel_pub_rate = rospy.Rate(10)



	def calculate_bias(self):
		self.bias_x = self.pose.position.x
		self.bias_y = self.pose.position.y
		self.bias_z = self.pose.position.z

		self.bias_roll = self.pose.orientation.x
		self.bias_pitch = self.pose.orientation.y
		self.bias_yaw = self.pose.orientation.z	

	def takeoff(self):
		self.takeoff_pub.publish()
		self.inFlight = True

	def land(self):
		self.land_pub.publish()
		self.inFlight = False

	def odom_callback(self, data):
		self.pose = data.pose.pose
		# print "pose ", self.pose
		self.Twist = data.twist.twist
		self.odom_data = data
		print("odom linear, is it array?", data.pose.pose.position)
		# print "twist ", self.Twist


	def routine(self):
		if self.image is not None:
			processed_img = preprocess_img(self.image)

			#run GMM inference to generate mask
			mask = test_combined(processed_img,self.K,self.n,self.weights, self.params,(0,255,0))
			area = np.count_nonzero(mask)
			self.areas.append(area)
			self.odom_list.append(self.odom_data)
			return True
		else:
			rospy.loginfo("got no image ")
			return False
	def orient(self, twist_msg):
		self.velocity_msg = twist_msg
		self.bebop_vel_pub.publish(self.velocity_msg)

	def yaw_cc(self):
		vel = Twist()
		vel.linear.x = 0
		vel.linear.y = 0
		vel.linear.z = 0

		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = -np.pi/60
		self.orient(vel)



# # *********************************************************
# perfect open loop
# def yaw_c(self):
# 		vel = Twist()
# 		vel.linear.x = 0
# 		vel.linear.y = 0
# 		vel.linear.z = 0

# 		vel.angular.x = 0
# 		vel.angular.y = 0
# 		# vel.angular.z = 
# 		ref_yaw= np.pi
# 		t = 0
# 		dt = 0.1
# 		while ((not rospy.is_shutdown()) and t <= ref_yaw):

# 			vel.angular.z = dt*6
# 			print("angular z = ", vel.angular.z)
# 			self.orient(vel)
# 			print("t = ",t)
# 			t+=dt
# 			self.vel_pub_rate.sleep()
# *********************************************************



	def move(self,ref):
		vel = Twist()
		# vel.angular.z = 
		ref_yaw= np.pi
		t = 0
		dt = 0.1
		while ((not rospy.is_shutdown()) and t <= ref_yaw):

			vel.linear.x = 0
			vel.linear.y = 0
			vel.linear.z = 0

			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = dt*6

			print("angular z = ", vel.angular.z)
			self.orient(vel)
			print("t = ",t)
			t+=dt
			self.vel_pub_rate.sleep()

	def yaw_c(self):
		vel = Twist()
		# vel.angular.z = 
		ref_yaw= np.pi
		t = 0
		dt = 0.1
		while ((not rospy.is_shutdown()) and t <= ref_yaw):

			vel.linear.x = 0
			vel.linear.y = 0
			vel.linear.z = 0

			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = dt*6

			print("angular z = ", vel.angular.z)
			self.orient(vel)
			print("t = ",t)
			t+=dt
			self.vel_pub_rate.sleep()

	def compute_vel_pid(self,ref_yaw,ref):
		current_state_yaw = self.odom_data.pose.pose.orientation.z - self.bias_yaw
		current_state_z = self.odom_data.pose.pose.position.z - self.bias_z
		err_yaw = ref_yaw - current_state_yaw
		vel_out = self.Kp*err_yaw
		# vel_out_yaw = self.Kp[5]*err_yaw
		return vel_out_yaw

	def find_max_area(self):
		if self.inFlight != True:
			print("in flight")
			self.takeoff()
			time.sleep(3)
			self.calculate_bias()
			# self.yaw_cc()
			time.sleep(3)
			self.yaw_c()

		

		# Task
			# capture image
			# apply gmm and get mask
			# calculate  area
			# save vel_msg and area
		#  change orientation 
		# repeat task

		# retrieve max area vel_msg
		

if __name__ == '__main__':
	rospy.init_node('align_quad', anonymous=True)
	pid = allign_quad()
	# rospy.on_shutdown(pid.land())
	pid.find_max_area()
	# sin_xy()
	# rhombus_traj()
	# sawtooth_traj()