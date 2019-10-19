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
# from GMM.test_data import *
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class moveit:
	def __init__(self):
		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		self.bebop_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1 , latch=True)
		self.vel_sp_sub = rospy.Subscriber('/cmd_vel_setpoint', Pose, self.vel_sp_cb)

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
		self.vel_sp = Pose()


	def vel_sp_cb(self,data):
		self.vel_sp = data

	def orient(self, twist_msg):
		self.velocity_msg = twist_msg
		self.bebop_vel_pub.publish(self.velocity_msg)

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


def main():



if __name__ == '__main__':
	main()