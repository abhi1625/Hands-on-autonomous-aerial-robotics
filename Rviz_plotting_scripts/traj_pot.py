#!/usr/bin/env python 
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose, PoseArray

class traj_plot:

	def __init__(self):
		self.topic = 'viz_traj'
		self.pose_topic = 'pose_traj'
		self.pose_pub = rospy.Publisher(self.pose_topic, PoseArray, queue_size = 50)
		self.MARKERS_MAX = 5
		
	def plot(self):
		count = 1
		rate = rospy.Rate(10)
		poseArr = PoseArray()
		while not rospy.is_shutdown():
			if count>self.MARKERS_MAX:
				count = 1
			pose = Pose()
			poseArr.header.frame_id = "/base"
			poseArr.header.stamp = rospy.Time()
			random_nos = np.random.randint(low=-5, high=self.MARKERS_MAX, size=6)
			pose.position.x = random_nos[0]
			pose.position.y = random_nos[1]
			pose.position.y = random_nos[2]

			pose.orientation.w = 1.0
			pose.orientation.x = random_nos[3]
			pose.orientation.y = random_nos[4]
			pose.orientation.z = random_nos[5]

			if(len(poseArr.poses)>self.MARKERS_MAX):
				poseArr.poses.pop(0)

			poseArr.poses.append(pose)
			self.pose_pub.publish(poseArr)

			print count
			count+=1
			rospy.sleep(0.05)



		


def main():
	
	rospy.init_node('traj_plot')
	obj = traj_plot()
	obj.plot()

if __name__ == '__main__':
	main()