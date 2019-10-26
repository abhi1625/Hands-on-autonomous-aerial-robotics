#!/usr/bin/env python

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

class trajectory_track:
	def __init__(self):
		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		self.bebop_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1 , latch=True)
		

		self.current_state = np.zeros((6,))
		self.prev_vel_odom = np.zeros((2,1))
		self.curr_vel_odom = np.zeros((2,1))
		self.curr_vel_yaw = 0.0 
		# current and previous position in x-y
		self.next_des = np.zeros((2,))
		self.prev_des = np.zeros((2,))
		
		# states for trajectory tracking
		self.pos_perpendicular = np.zeros((2,1))
		self.vel_perpendicular = np.zeros((2,1))
		self.vel_parallel = np.zeros((2,1))
		self.acc_parallel = np.zeros((2,1))


	def odom_callback(self, data):
		self.current_state[0] = data.twist.twist.linear.x*0.2 + self.current_state[0]
		self.current_state[1] = data.twist.twist.linear.y*0.2 + self.current_state[1]
		self.current_state[2] = data.pose.pose.position.z
		self.current_state[3] = data.pose.pose.orientation.x
		self.current_state[4] = data.pose.pose.orientation.y
		self.current_state[5] = data.twist.twist.angular.z*0.2 + self.current_state[5]

		self.curr_vel_odom[0] = data.twist.twist.linear.x
		self.curr_vel_odom[1] = data.twist.twist.linear.y 
		self.curr_vel_yaw = data.twist.twist.angular.z
		# self.odom_data = data

	def takeoff(self):
		self.takeoff_pub.publish()
		# self.inFlight = True

	def land(self):
		self.land_pub.publish()
		# self.inFlight = False

	def traj_gen(self):
		if ((self.next_des[0] == 0) and (self.next_des[1] == 0) and
			 (self.prev_des[0] == 0) and (self.prev_des[1] == 0)) :
			 l_parallel = np.array([0,0])
			 l_perpendicular = np.array([0,0])
		else:
			norm = (1/np.linalg.norm(self.next_des.reshape(-1,1) - self.prev_des.reshape(-1,1)))
			l_parallel = norm*np.array([[self.next_des[0] - self.prev_des[0]],
										[self.next_des[1] - self.prev_des[1]]])
			l_perpendicular = norm*np.array([[-self.next_des[1] + self.prev_des[1]],
											 [self.next_des[0] - self.prev_des[0]]])
		return l_parallel, l_perpendicular	

	def compute_states(self, l_parallel, l_perpendicular):
		# print(l_parallel.shape, (self.curr_vel_odom - self.prev_vel_odom).shape)
		self.acc_parallel = np.matmul(l_parallel.T, (self.curr_vel_odom - self.prev_vel_odom))
		self.vel_parallel = np.matmul(l_parallel.T, self.curr_vel_odom)
		self.vel_perpendicular = np.matmul(l_perpendicular.T, self.curr_vel_odom)
		pos = np.array([[self.current_state[0] - self.prev_des[0]],
						[self.current_state[1] - self.prev_des[1]]])
		self.pos_perpendicular = np.matmul(l_perpendicular.T, pos)
		pos_dist = np.array([[self.next_des[0] - self.current_state[0]],
				     [self.next_des[0] - self.current_state[1]]])
		pos_parallel = np.matmul(l_parallel.T, pos_dist)
		print("distance to dest: ",pos_parallel)

	def compute_ctrl_inputs(self, l_parallel, l_perpendicular):
		K_line = np.array([[0.2207, 0.2634, 	 0, 	0],
						   [	 0, 	 0, 0.1301, 0.065]])
		
		states = np.array([self.pos_perpendicular, 
						   self.vel_perpendicular,
						   self.vel_parallel,
						   self.acc_parallel])
		states = states.reshape(4,1)
		ctrl_inputs = - np.matmul(K_line,states)
		# print(l_perpendicular.shape)
		# ctrl_inputs_gf = l_parallel*ctrl_inputs[0] + l_perpendicular*ctrl_inputs[1]
		ctrl_inputs_gf = np.zeros((2,))
		gains = np.array([[0.2236, 0.2657]])

		# print(gains)
		x_pos = np.array([[self.current_state[0] - self.next_des[0]],
				  self.curr_vel_odom[0]])
		# print(x_pos)
		y_pos = np.array([[self.current_state[1] - self.next_des[1]],
				  self.curr_vel_odom[1]])
		ctrl_inputs_gf[0] = - np.matmul(gains, x_pos)
		ctrl_inputs_gf[1] = - np.matmul(gains, y_pos)
		return ctrl_inputs_gf

def frame_transform(self,pos_camera):
	Tf_quad = np.array([[0, 0, -1],
					   [-1, 0, 0],
					   [0, 1,  0]])
	Tf_inertial = np.array([[1,0,0, self.current_state[0]],
							[0,1,0, self.current_state[1]],
							[0,0,1, self.current_state[2]],
							[0,0,0,						1]])

	pos_quad = np.matmul(Tf_mat, pos_camera)
	pos_quad = np.array([[pos_quad[0]],
						 [pos_quad[1]],
						 [pos_quad[2]],
						 [		1.0 ]])
	pos_inertial = np.matmul(Tf_inertial, pos_quad)
	return pos_quad

def main():
	rospy.init_node('trajectory_following', anonymous=True)
	track_ob = trajectory_track()
	track_ob.takeoff()
	rospy.sleep(5)
	init_flag = True
	rate = rospy.Rate(10)
	while (not rospy.is_shutdown()):
		if init_flag :
			bias_ang_z = track_ob.current_state[5]

			init_flag = False
		#track_ob.current_state = track_ob.current_state - np.array([bias_x,bias_y,bias_z,bias_ang_x,bias_ang_y,bias_ang_z]) 
		print("current x:",track_ob.current_state[0])
		print("current y:",track_ob.current_state[1])
		#) print(track_ob.current_state)
		track_ob.next_des = np.array([1,0])
		l_parallel, l_perpendicular = track_ob.traj_gen()
		yaw_reference = math.atan2(l_parallel[1], l_parallel[0])
		#print("yaw reference",yaw_reference)
		track_ob.compute_states(l_parallel, l_perpendicular)	
		ctrl_inputs = track_ob.compute_ctrl_inputs(l_parallel,l_perpendicular)
		vel = Twist()
		vel.linear.x = 1*ctrl_inputs[0]
		vel.linear.y = 1*ctrl_inputs[1]
		vel.linear.z = 0
			
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 1*(yaw_reference - track_ob.current_state[5])
		# vel.angular.z = 
		print("vel z",vel.linear.z )
		print("vel y", vel.linear.y)
		track_ob.bebop_vel_pub.publish(vel)
		rate.sleep()

if __name__ == '__main__':
	main()
