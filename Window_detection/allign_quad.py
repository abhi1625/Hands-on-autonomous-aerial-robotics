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
from window_detection import *
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
		self.vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1 , latch=True)
		self.cam_pose_sub = rospy.Subscriber('/relative_pose',Twist,self.cam_pose_cb)
		
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
		self.Kp = [1.5, 1.5, 1.5,0,0,1]
		self.inFlight = False
		self.vel_pub_rate = rospy.Rate(10)
		self.height_offset = 1
		self.y_thresh = 0.2
		self.z_thresh = 0.2
		self.yaw_thresh = 0.05
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.yaw = 0.0


	def cam_pose_cb(self,data):
		self.x = data.linear.x
		self.y = data.linear.y
		self.z = data.linear.z
		self.yaw = data.angular.z
	def calculate_bias(self):
		self.bias_x = self.pose.position.x
		self.bias_y = self.pose.position.y
		self.bias_z = self.pose.position.z

		self.bias_roll = self.pose.orientation.x
		self.bias_pitch = self.pose.orientation.y
		self.bias_yaw = self.pose.orientation.z	

	def takeoff(self):
		if self.inFlight != True:
			self.takeoff_pub.publish()
			print("in flight")
			time.sleep(3)
			self.inFlight = True

	def land(self):
		self.land_pub.publish()
		self.inFlight = False

	def odom_callback(self, data):
		self.pose = data.pose.pose
		# print "pose ", self.pose
		self.Twist = data.twist.twist
		self.odom_data = data
		# print("odom linear, is it array?", data.pose.pose.position)
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
		self.vel_pub.publish(self.velocity_msg)



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

	def max_pts(self,):
		img = self.vs.get_img()
		img[img[:,:,0]<2 and img[:,:,0]>237] = 0 
		img[img[:,:,0]<74 and img[:,:,0]>237] = 208
		img[img[:,:,0]<72 and img[:,:,0]>237] = 100
		area = np.count_nonzero(img)
		return area 
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

	def yaw_c(self,angle,duration):
		vel = Twist()
		# vel.angular.z = 
		ref_yaw= angle
		t = 0
		dt = 0.1
		while ((not rospy.is_shutdown()) and t <= duration):

			vel.linear.x = 0
			vel.linear.y = 0
			vel.linear.z = 0

			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = -dt*self.Kp[-1]

			# print("angular z = ", vel.angular.z)
			self.orient(vel)
			print("t = ",t)
			t+=dt
			self.vel_pub_rate.sleep()

	def yaw_cc(self,angle,duration):
		vel = Twist()
		# vel.angular.z = 
		ref_yaw= angle
		t = 0
		dt = 0.1
		while ((not rospy.is_shutdown()) and t <= duration):

			vel.linear.x = 0
			vel.linear.y = 0
			vel.linear.z = 0

			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = dt*self.Kp[-1]

			# print("angular z = ", vel.angular.z)
			self.orient(vel)
			print("t = ",t)
			t+=dt
			self.vel_pub_rate.sleep()

	def move_up(self, height):
		vel = Twist()
		t = 0
		dt = 0.1
		while ((not rospy.is_shutdown()) and t <= height):

			vel.linear.x = 0
			vel.linear.y = 0
			vel.linear.z = dt*self.Kp[2]

			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = 0

			print("angular z = ", vel.angular.z)
			self.orient(vel)
			print("t = ",t)
			t+=dt
			self.vel_pub_rate.sleep()

	# def compute_vel_pid(self,ref_yaw,ref):
	# 	current_state_yaw = self.odom_data.pose.pose.orientation.z - self.bias_yaw
	# 	current_state_z = self.odom_data.pose.pose.position.z - self.bias_z
	# 	err_yaw = ref_yaw - current_state_yaw
	# 	vel_out = self.Kp*err_yaw
	# 	# vel_out_yaw = self.Kp[5]*err_yaw
	# 	return vel_out_yaw


	def open_loop_align(self,y,z,yaw):
		vel = Twist()
		t = 0
		dt = 0.1
		# self.ref_y
		# self.ref_z
		# self.ref_yaw
		while ((not rospy.is_shutdown()) and t <= 0.4):

			vel.linear.x = 0
			if y == 1:
				vel.linear.y = dt*self.Kp[1]
			elif y == -1:
				vel.linear.y = -dt*self.Kp[1]
			else:
				vel.linear.y = 0
			if z == 1:
				vel.linear.z = dt*self.Kp[2]
			elif z == -1:
				vel.linear.z = -dt*self.Kp[2]
			else:
				vel.linear.z = 0

			if yaw == 1:
				vel.angular.z = dt*self.Kp[-1]
			if yaw == -1:
				vel.angular.z = -dt*self.Kp[-1]
			else:
				vel.angular.z = 0
			
			vel.angular.y = 0
			vel.angular.x = 0

			# print("angular z = ", vel.angular.z)
			self.orient(vel)
			# print("t = ",t)
			t+=dt
			self.vel_pub_rate.sleep()

	def dir_check(self):
		if(self.y>0):
			y = 1
		elif(self.y<0):
			y = -1
		else:
			y = 0

		if(self.z>0):
			z = 1
		elif(self.z<0):
			z = -1
		else:
			z = 0

		if(self.yaw>0):
			yaw = 1
		elif(self.yaw<0):
			yaw = -1
		else:
			yaw = 0
		return y,z,yaw

	def align_y_z_yaw(self):

		y,z,yaw = self.dir_check()

		self.open_loop_align(y,z,yaw)
		time.sleep(3)
		dt = 0
		t = 0
		while((abs(self.yaw)<abs(self.yaw_thresh) and abs(self.y)<abs(self.y_thresh)) or t < 10 or (not rospy.is_shutdown())):
			if dt>=5:
				time.sleep(3)
				dt = 0
				t =+ 1
			print("y,z,yaw",self.y,self.z,self.yaw)
			y,z,yaw = self.dir_check()
			self.open_loop_align(y,z,yaw)
			time.sleep(3)
			dt+=0.1
		print("align ended")		
			
		

	def find_max_area(self):
			self.calculate_bias()
			time.sleep(1)
			self.yaw_c(np.pi/60,0.2)
			print("done yaw c")
			time.sleep(1)
			self.yaw_cc(np.pi/30,0.4)
			time.sleep(2)
			self.move_up(0.6)
			time.sleep(1)
			self.yaw_c(np.pi/60,0.2)
			time.sleep(1)
			self.yaw_c(np.pi/60,0.2)

	
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
	pid.takeoff()
	# rospy.on_shutdown(pid.land())
	pid.align_y_z_yaw()
	# sin_xy()
	# rhombus_traj()
	# sawtooth_traj()