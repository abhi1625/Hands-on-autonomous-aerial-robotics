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




# class moveit:
# 	def __init__(self):
# 		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
# 		self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1 , latch=True)
# 		self.bebop_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
# 		self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1 , latch=True)
# 		self.cam_pose_sub = rospy.Subscriber('/relative_pose',Twist,self.cam_pose_cb)
# 		self.vision_status_sub = rospy.Subscriber('/vision_status',String,self.vision_status_cb)
# 		self.inFlight = False
# 		self.pose = Pose()
# 		# self.twist = Twist()
# 		self.bias = np.zeros((6,))
# 		self.velocity_msg = Twist()
# 		self.areas = []
# 		self.odom_list = []
# 		self.dt = 0.1
# 		self.inFlight = False
# 		self.vel_pub_rate = rospy.Rate(10)
# 		self.current_state = np.zeros((6,))
# 		self.x_sp = 0
# 		self.y_sp = 0
# 		self.z_sp = 1
# 		self.yaw_sp = 0
# 		self.min_step = 0.0
# 		self.max_step = 0.4
# 		self.Kp = [0.02,0.03,0.3,0.0,0.0,0.04]
# 		self.vision_status = "inactive"

# 	def vision_status_cb(self,data):
# 		self.vision_status = data.data

# 	def cam_pose_cb(self,data):
# 		#print("ya campose", data.angular.z)
# 		self.x_sp = data.linear.x
# 		self.y_sp = data.linear.y
# 		self.z_sp = data.linear.z
# 		self.yaw_sp = data.angular.z


# 	def orient(self, twist_msg):
# 		self.velocity_msg = twist_msg
# 		self.bebop_vel_pub.publish(self.velocity_msg)

# 	def calculate_bias(self):
# 		self.bias[0] = self.pose.position.x
# 		self.bias[1] = self.pose.position.y
# 		self.bias[2] = self.pose.position.z
# 		self.bias[3] = self.pose.orientation.x
# 		self.bias[4] = self.pose.orientation.y
# 		self.bias[5] = self.pose.orientation.z
		
# 	def takeoff(self):
# 		self.takeoff_pub.publish()
# 		self.inFlight = True

# 	def land(self):
# 		self.land_pub.publish()
# 		self.inFlight = False

# 	def odom_callback(self, data):
# 		self.pose = data.pose.pose
# 		# print "pose ", self.pose
# 		# self.Twist = data.twist.twist
# 		self.current_state[0] = data.pose.pose.position.x
# 		self.current_state[1] = data.pose.pose.position.y
# 		self.current_state[2] = data.pose.pose.position.z
# 		self.current_state[3] = data.pose.pose.orientation.x
# 		self.current_state[4] = data.pose.pose.orientation.y
# 		self.current_state[5] = data.pose.pose.orientation.z
# 		self.odom_data = data
# 		# print("odom linear, is it array?", data.pose.pose.position)
# 		# print "twist ", self.Twist


# 	# 		# print("angular z = ", vel.angular.z)
# 	# 		self.orient(vel)
# 	# 		# print("t = ",t)
# 	# 		t+=dt
# 	# 		self.vel_pub_rate.sleep()

# 	def punch_through(self):
# 		t = 0
# 		dt = 0.1
# 		vel = Twist()
# 		while(t<1.3 or (not rospy.is_shutdown())):
# 			vel.linear.x = dt
# 			vel.linear.y = 0
# 			vel.linear.z = 0
				
# 			vel.angular.x = 0
# 			vel.angular.y = 0
# 			vel.angular.z = 0
# 			t += dt
				
# 			self.bebop_vel_pub.publish(vel)

# 	def move_up(self, height):
# 		vel = Twist()
# 		t = 0
# 		dt = 0.1
# 		while ((not rospy.is_shutdown()) and t <= height):

# 			vel.linear.x = 0
# 			vel.linear.y = 0
# 			vel.linear.z = dt*1.5

# 			vel.angular.x = 0
# 			vel.angular.y = 0
# 			vel.angular.z = 0

# 			#print("angular z = ", vel.angular.z)
# 			self.orient(vel)
# 			#print("t = ",t)
# 			t+=dt
# 			self.bebop_vel_pub.publish(vel)



# 	def move(self,ref):

# 		vel = Twist()
# 		# dt = 0.1
# 		# print("reference state = ", ref[:3])
# 		# print("current_state = ", abs_curr_state[:3])
# 		# err = ref + abs_curr_state

# 		#print("ref = ", ref)
# 		current_abs_state = self.current_state - self.bias
# 		#print("biased compensated state",current_abs_state[2])
# 		err_x = ref[0] - current_abs_state[0]
# 		vel_arr_x = self.Kp[0]*err_x
		
# 		#print("vel x: ",vel_arr_x)		
# 		err_y = ref[1] - current_abs_state[1]
# 		vel_arr_y = self.Kp[1]*err_y
# 		#print("ref z= ",ref[2])
# 		err_z = ref[2] - current_abs_state[2]
# 		#print("err z= ",err_z)
# 		vel_arr_z = self.Kp[2]*err_z
# 		#print("vel z: ",vel_arr_z)
				
# 		err_yaw = -ref[-1] + current_abs_state[-1]
# 		#print("err_yaw = ", err_yaw)
# 		vel_arr_yaw = self.Kp[-1]*err_yaw
# 		#print("vel yaw",vel_arr_yaw)
# 		#print("kp yaw = ", self.Kp[-1])

# 		vel.linear.x = vel_arr_x
# 		vel.linear.y = vel_arr_y
# 		vel.linear.z = vel_arr_z

# 		vel.angular.x = 0
# 		vel.angular.y = 0
# 		vel.angular.z = vel_arr_yaw
# 		# print("angular z = ", vel.angular.z)
# 		self.orient(vel)
# 		# print("t = ",t)
# 		# t+=dt
# 		self.vel_pub_rate.sleep()

# 	def iLikeToMoveItMoveIt(self):
# 		ref = np.array([self.x_sp,self.y_sp,self.z_sp,self.yaw_sp])
# 		while (not rospy.is_shutdown()):
# 			#print(ref)
# 			curr = self.current_state - self.bias
# 			if(self.vision_status == "active" or ref[0]>1):
# 				print("update ref")
# 				print("curr = ",curr)
# 				print("prev ref = ",ref)
# 				ref = np.array([self.x_sp+curr[0],self.y_sp+curr[1],curr[2]+self.z_sp,(-self.y_sp-curr[1])*0.1])
# 				print("new ref = ",ref)
# 				self.move(ref)
# 			elif(ref[0]<=1.0 and ref[0] != 0.0):
# 				print("punching")
# 				print("curr = ",curr)
# 				print("new ref = ",ref)
# 				self.punch_through()
# 			print("waiting for vision sp")
# # def land_out():
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
		# self.pose = data.pose.pose
		# print "pose ", self.pose
		# self.Twist = data.twist.twist
		self.current_state[0] = data.twist.twist.linear.x*0.2 + self.current_state[0]
		self.current_state[1] = data.twist.twist.linear.y*0.2 + self.current_state[1]
		self.current_state[2] = data.pose.pose.position.z
		self.current_state[3] = data.pose.pose.orientation.x
		self.current_state[4] = data.pose.pose.orientation.y
		self.current_state[5] = data.pose.pose.orientation.z

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

		print(gains)
		x_pos = np.array([[self.current_state[0] - self.next_des[0]],
				  self.curr_vel_odom[0]])
		print(x_pos)
		y_pos = np.array([[self.current_state[1] - self.next_des[1]],
				  self.curr_vel_odom[1]])
		ctrl_inputs_gf[0] = - np.matmul(gains, x_pos)
		ctrl_inputs_gf[1] = - np.matmul(gains, y_pos)
		return ctrl_inputs_gf


def main():
	rospy.init_node('trajectory_following', anonymous=True)
	track_ob = trajectory_track()
	track_ob.takeoff()
	rospy.sleep(5)
	init_flag = True
	rate = rospy.Rate(10)
	while (not rospy.is_shutdown()):
		if init_flag :
			bias_x = track_ob.current_state[0]
			bias_y = track_ob.current_state[1]
			bias_z = track_ob.current_state[2]
			bias_ang_x = track_ob.current_state[3]
			bias_ang_y = track_ob.current_state[4]
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
		#vel.angular.z = 0.1*(yaw_reference - track_ob.current_state[5]) - 0.01*track_ob.curr_vel_yaw 
		vel.angular.z = 0
		print("vel x",vel.linear.x )
		print("vel y", vel.linear.y)
		track_ob.bebop_vel_pub.publish(vel)
		rate.sleep()

if __name__ == '__main__':
	main()
