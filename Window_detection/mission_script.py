#!/usr/bin/env python

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import time
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged

class trajectory_track:
	def __init__(self):
		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
		self.bebop_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1 , latch=True)
		self.rotation_sub = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',Ardrone3PilotingStateAttitudeChanged, self.rot_callback)
		self.target_sub = rospy.Subscriber('/cctag_sp', Pose, self.pose_cb)
		self.vision_pub = rospy.Publisher('/cctag_detect', Bool, queue_size=1)
		self.target_sp = Pose()
		self.target = np.zeros((3,))

		self.current_state = np.zeros((6,))
		self.prev_vel_odom = np.zeros((2,1))
		self.curr_vel_odom = np.zeros((2,1))
		self.curr_vel_yaw = 0.0 
		# current and previous position in x-y
		self.x_coord = 3.1
		self.y_coord = 1.7
		self.next_des = np.array([self.x_coord, self.y_coord])
		self.prev_des = np.zeros((2,))
		
		# states for trajectory tracking
		self.pos_perpendicular = np.zeros((2,1))
		self.vel_perpendicular = np.zeros((2,1))
		self.vel_parallel = np.zeros((2,1))
		self.acc_parallel = np.zeros((2,1))

	def pose_cb(self,data):
		self.target_sp = data
		self.target[0] = data.position.x
		self.target[1] = data.position.y
		self.target[2] = data.position.z
		self.target.reshape((3,1))

	def odom_callback(self, data):
		# self.pose = data.pose.pose
		# print "pose ", self.pose
		# self.Twist = data.twist.twist
		self.current_state[0] = data.twist.twist.linear.x*0.2 + self.current_state[0]
		self.current_state[1] = data.twist.twist.linear.y*0.2 + self.current_state[1]
		self.current_state[2] = data.twist.twist.linear.z*0.2 + self.current_state[2]

		self.curr_vel_odom[0] = data.twist.twist.linear.x
		self.curr_vel_odom[1] = data.twist.twist.linear.y 
		self.curr_vel_yaw = data.twist.twist.angular.z
		# self.odom_data = data
	def rot_callback(self, data):
		self.current_state[3] = data.roll
		self.current_state[4] = data.pitch
		self.current_state[5] = data.yaw

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

	def compute_ctrl_inputs(self):
		K_line = np.array([[0.2207, 0.2634, 	 0, 	0],
						   [	 0, 	 0, 0.1301, 0.065]])
		
		#states = np.array([self.pos_perpendicular, 
		#				   self.vel_perpendicular,
		#				   self.vel_parallel,
		#				   self.acc_parallel])
		#states = states.reshape(4,1)
		#ctrl_inputs = - np.matmul(K_line,states)
		# print(l_perpendicular.shape)
		# ctrl_inputs_gf = l_parallel*ctrl_inputs[0] + l_perpendicular*ctrl_inputs[1]
		ctrl_inputs_gf = np.zeros((2,))
		gains = np.array([[0.2236, 0.2657]])

		#print(gains)
		x_pos = np.array([[self.current_state[0] - self.next_des[0]],
				  self.curr_vel_odom[0]])
		#print(x_pos)
		y_pos = np.array([[self.current_state[1] - self.next_des[1]],
				  self.curr_vel_odom[1]])
		ctrl_inputs_gf[0] = - np.matmul(gains, x_pos)
		ctrl_inputs_gf[1] = - np.matmul(gains, y_pos)
		return ctrl_inputs_gf

	def frame_transform(self,pos_camera):
		Tf_quad = np.array([[0, -1, 0],
					   [1, 0, 0],
					   [0, 0,  1]])
		Tf_inertial = np.array([[1,0,0, self.current_state[0]],
							[0,1,0, self.current_state[1]],
							[0,0,1, self.current_state[2]],
							[0,0,0,						1]])

		pos_quad = np.matmul(Tf_quad, pos_camera)
		pos_quad = np.array([[pos_quad[0]],
						 [pos_quad[1]],
						 [pos_quad[2]],
						 [		1.0 ]])
		pos_inertial = np.matmul(Tf_inertial, pos_quad)
		return pos_inertial

def main():
	rospy.init_node('trajectory_following', anonymous=True)
	track_ob = trajectory_track()
	track_ob.takeoff()
	rospy.sleep(5)
	init_flag = True
	rate = rospy.Rate(10)
	# enter center coordinates
	#track_ob.next_des = np.array([4.3,-1.9])
	x_detection = 100
	y_detection = 100
	vel = Twist()
	detection_status = False
	mission1 = True
	mission2 = False
	mission3 = False
	while (not rospy.is_shutdown()):
		#print(track_ob.target_sp)
		if init_flag :
			bias_x = track_ob.current_state[0]
			bias_y = track_ob.current_state[1]
			bias_z = track_ob.current_state[2]
			bias_ang_x = track_ob.current_state[3]
			bias_ang_y = track_ob.current_state[4]
			bias_ang_z = track_ob.current_state[5]

			init_flag = False
		#track_ob.current_state[5] = track_ob.current_state[5] - bias_ang_z 
		#print("current z:",track_ob.current_state[2])
		#print("current y:",track_ob.current_state[1])
		#) print(track_ob.current_state)
		track_ob.vision_pub.publish(detection_status)
		#l_parallel, l_perpendicular = track_ob.traj_gen()
		#yaw_reference = math.atan2(l_parallel[1], l_parallel[0]) + bias_ang_z
		yaw_reference = 0.0
		if yaw_reference > math.pi:
			yaw_reference = math.pi - yaw_reference
		elif yaw_reference < -math.pi:
			yaw_reference = -math.pi - yaw_reference
		print("x_vel", vel.linear.x)
		print("y_vel", vel.linear.y)
		#track_ob.compute_states(l_parallel, l_perpendicular)	
		ctrl_inputs = track_ob.compute_ctrl_inputs()
		#print("x",track_ob.current_state[0],"y", track_ob.current_state[1])
		track_ob.bebop_vel_pub.publish(vel)
		if (mission1):
			vel.linear.x = 1*ctrl_inputs[0]
			vel.linear.y = 1*ctrl_inputs[1]
		elif(mission2):
			vel.linear.x = 0.5*ctrl_inputs[0]
			vel.linear.y = 0.5*ctrl_inputs[1]
			if vel.linear.x > 0.2:
				vel.linear.x = 0.2
			if vel.linear.y > 0.2:
				vel.linear.y = 0.2
			if vel.linear.x < -0.2:
				vel.linear.x = -0.2
			if vel.linear.y < -0.2:
				vel.linear.y = -0.2
			#if (abs(x_detection - track_ob.current_state[0]) < 0.5) :
			#	vel.linear.x = 0
			
			#if (abs(y_detection - track_ob.current_state[1]) < 0.5) :
			#	vel.linear.y = 0
			print("vel_x ", vel.linear.x,"vel_y ", vel.linear.y) 
		elif(mission3):
			vel.linear.x = 3*ctrl_inputs[0]
			vel.linear.y = 3*ctrl_inputs[1]
		vel.linear.z = 0	
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0.0*(yaw_reference - track_ob.current_state[5])
	
		if (mission1 and (track_ob.x_coord-0.1 < track_ob.current_state[0] < track_ob.x_coord+0.1) and (track_ob.y_coord-0.1< track_ob.current_state[1] < track_ob.y_coord+0.1)):
			#increase z
			detection_status = True
			track_ob.vision_pub.publish(detection_status)
			print("###################################")
			while(track_ob.current_state[2] < 1.0):
				#print('inloop')
				vel.linear.z = 0.3
				vel.linear.x = 0
				vel.linear.y = 0
				vel.angular.z = 0.0
				track_ob.bebop_vel_pub.publish(vel)

			vel.linear.z = 0.0
			#rospy.sleep(2)
			pos_quad = track_ob.frame_transform(track_ob.target)
			x_detection = pos_quad[0,0]
			y_detection = pos_quad[1,0]
			track_ob.next_des = np.array([pos_quad[0,0], pos_quad[1,0]])
			mission1 = False
			mission2 = True
			# detect and align (x,y) with circle center
		
			# track_ob.next_des = np.array([track_ob.target[0], track_ob.target[1])
		if (mission2 and (x_detection-0.1 < track_ob.current_state[0] < x_detection+0.1) and (y_detection-0.1 < track_ob.current_state[1] < y_detection+0.1)):
				
			while(track_ob.current_state[2] > 1.0):
				#print('inloop')
				vel.linear.z = -0.3
				vel.linear.x = 0
				vel.linear.y = 0
				vel.angular.z = 0.0
				track_ob.bebop_vel_pub.publish(vel)
			rospy.sleep(2)
			pos_quad = track_ob.frame_transform(track_ob.target)
			x_detection2 = pos_quad[0,0]
			y_detection2 = pos_quad[1,0]
			track_ob.next_des = np.array([pos_quad[0,0], pos_quad[1,0]])
			mission3 = True
			mission2 = False
		if (mission3 and (x_detection2-0.05 < track_ob.current_state[0] < x_detection2+0.05) and (y_detection2-0.05< track_ob.current_state[1] < y_detection2+0.05)):		
			vel.linear.z = 0.0
			vel.linear.x = 2*ctrl_inputs[0]
			vel.linear.y = 5*ctrl_inputs[1]
			#track_ob.next_des = np.array([x_detection, y_detection])
			print("Landing")
			rospy.sleep(2)	
			track_ob.land()
		#print("pos quad = ", pos_quad)
		#print("yaw reference",yaw_reference)
		# vel.angular.z = 
		#print("vel z",vel.angular.z )
		#print("vel y", vel.linear.y)i
		print("traj ",track_ob.next_des)
		rate.sleep()

if __name__ == '__main__':
	main()
