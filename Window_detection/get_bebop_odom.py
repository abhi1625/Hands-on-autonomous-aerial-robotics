#!/usr/bin/env python

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, Pose, PoseArray
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import ColorRGBA
import tf.transformations
from nav_msgs.msg import Odometry
# from GMM.test_data import *
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class moveit:
	def __init__(self):
		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
		self.bias = np.zeros((7,))
		self.current_state = np.zeros((7,))
		self.pose_pub = rospy.Publisher('/pose_traj', PoseArray, queue_size = 50)
		self.MARKERS_MAX = 5
		self.topic = 'viz_traj'
		# self.pose_topic = 'pose_traj'
		self.marker_pub = rospy.Publisher(self.topic, MarkerArray, queue_size = 50)
		self.marker_array = MarkerArray()
		self.pose = Pose()



	def calculate_bias(self):
		# self.pose = data.pose.pose
		self.bias[0] = self.pose.position.x + 0.3
		self.bias[1] = self.pose.position.y - 4.5
		self.bias[2] = self.pose.position.z - 1.0
		self.bias[3] = self.pose.orientation.w
		self.bias[4] = self.pose.orientation.x
		self.bias[5] = self.pose.orientation.y
		self.bias[6] = self.pose.orientation.z
		print("bias = ", self.bias)


	def odom_callback(self, data):
		self.pose = data.pose.pose
		# print "pose ", self.pose
		# self.Twist = data.twist.twist
		self.current_state[0] = data.pose.pose.position.x
		self.current_state[1] = data.pose.pose.position.y
		self.current_state[2] = data.pose.pose.position.z
		self.current_state[3] = data.pose.pose.orientation.x
		self.current_state[4] = data.pose.pose.orientation.x
		self.current_state[5] = data.pose.pose.orientation.y
		self.current_state[6] = data.pose.pose.orientation.z
		self.odom_data = data

	def plot(self):
		count = 1
		rate = rospy.Rate(10)
		poseArr = PoseArray()
		while not rospy.is_shutdown():
			if count>self.MARKERS_MAX:
				count = 1
			curr = self.current_state-self.bias
			self.plot_window()
			pose = Pose()
			poseArr.header.frame_id = "/base"
			poseArr.header.stamp = rospy.Time()
			pose.position.x = curr[0]
			pose.position.y = curr[1]
			pose.position.z = curr[2]

			pose.orientation.w = curr[3]
			pose.orientation.x = curr[4]
			pose.orientation.y = curr[5]
			pose.orientation.z = curr[6]

			if(len(poseArr.poses)>self.MARKERS_MAX):
				poseArr.poses.pop(0)

			poseArr.poses.append(pose)
			self.pose_pub.publish(poseArr)

			print count
			count+=1
			rospy.sleep(0.05)
	

	def create_marker(self,pos,angle,scale,ts,idd):
		marker = Marker()
		marker.color  = ColorRGBA(r=1., g=0 , b=0., a=1)
		marker.lifetime = rospy.Duration.from_sec(60)
		marker.header.frame_id = 'base'
		marker.id = idd
		marker.header.stamp= ts
		marker.type = Marker.CYLINDER
		marker.action = marker.ADD

		marker.pose.position.x = pos[0]
		marker.pose.position.y = pos[1]
		marker.pose.position.z = pos[2]
		quaternion = tf.transformations.quaternion_from_euler(angle[0], angle[1],angle[2])
		marker.pose.orientation.w = quaternion[0]
		marker.pose.orientation.x = quaternion[1]
		marker.pose.orientation.y = quaternion[2]
		marker.pose.orientation.z = quaternion[3]
		marker.scale.x = scale[0]
		marker.scale.y = scale[1]
		marker.scale.z = scale[2]
		self.marker_array.markers.append(marker)
		
		

	def plot_window(self):
		count = 1
		rate = rospy.Rate(10)

		# while not rospy.is_shutdown():
		print(len(self.marker_array.markers))

		bar_length = 1.0  #meter
		pole_length = 1.0 #meter
		stand_length = 0.5 #meter
		self.marker_array.markers = []
		timestamp = rospy.Time.now()
		# left leg
		base_pos = np.array([0.0,-bar_length/2,0.0])
		base_angle = np.array([1.57,1.57,1.57])
		scale = np.array([0.1,0.1,stand_length])
		print("base angle = ", base_angle[0],base_angle[1],base_angle[2])
		self.create_marker(base_pos,base_angle,scale,timestamp,1)

		# right leg
		scale = np.array([0.1,0.1,stand_length])
		pos = np.array([base_pos[0],base_pos[1]+1.0,base_pos[2]])
		angle = np.array([base_angle[0],base_angle[1],base_angle[2]])
		print("base angle = ", angle)
		self.create_marker(pos,angle,scale,timestamp,2)

		# left stand
		scale = np.array([0.1,0.1,pole_length])
		pos = np.array([base_pos[0],base_pos[1],base_pos[2]+scale[2]/2])
		angle = np.array([[base_angle[0]],[base_angle[1]-1.57],[base_angle[2]-1.57]])
		self.create_marker(pos,angle,scale,timestamp,3)

		# right stand
		scale = np.array([[0.1],[0.1],[1]])
		pos = np.array([[base_pos[0]],[base_pos[1]+bar_length],[base_pos[2]+scale[2]/2]])
		angle = np.array([[base_angle[0]],[base_angle[1]-1.57],[base_angle[2]-1.57]])
		self.create_marker(pos,angle,scale,timestamp,4)

		# upper bar
		scale = np.array([[0.1],[0.1],[bar_length]])
		pos = np.array([base_pos[0],base_pos[1]+bar_length/2,base_pos[2]+pole_length/2])
		angle = np.array([base_angle[0]+1.57,base_angle[1]-1.57,base_pos[2]-1.57])
		self.create_marker(pos,angle,scale,timestamp,5)
		self.marker_pub.publish(self.marker_array)

		# lower bar
		scale = np.array([[0.1],[0.1],[bar_length]])
		pos = np.array([base_pos[0],base_pos[1]+bar_length/2,base_pos[2]+pole_length])
		angle = np.array([base_angle[0]+1.57,base_angle[1]-1.57,base_pos[2]-1.57])
		self.create_marker(pos,angle,scale,timestamp,6)
		self.marker_pub.publish(self.marker_array)

		for i in range(len(self.marker_array.markers)):
			print("zpos of ",i," ",self.marker_array.markers[i].pose.orientation.x)
		# print count
		# count+=1
		# rospy.sleep(0.1)


def main():
	rospy.init_node('get_odom', anonymous=True)
	pos_hld = moveit()
	pos_hld.calculate_bias()
	pos_hld.plot()
	# pos_hld.xline()
	# rospy.on_shutdown(pid.land())

if __name__ == '__main__':
	main()
