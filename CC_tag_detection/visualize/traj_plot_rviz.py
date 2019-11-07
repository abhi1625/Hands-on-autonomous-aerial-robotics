#!/usr/bin/env python 
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose, PoseArray

class traj_plot:

	def __init__(self):
		self.topic = 'viz_traj'
		self.pose_topic = 'pose_traj'
		self.pose_pub = rospy.Subscriber("/quad_pose",Pose, self.traj_pose_cb)
		self.pose_pub = rospy.Publisher(self.pose_topic, PoseArray, queue_size = 50)
		self.MARKERS_MAX = 10
		self.curr_pose = Pose()

	def traj_pose_cb(self, data):
		self.curr_pose = data

	def plot(self):
		count = 1
		rate = rospy.Rate(30)
		poseArr = PoseArray()
		tf = np.array([[1,0.0,0.0,-3.0],[0.0,1.0,0.0,-2.4],[0.0,0.0,1.0,-0.20]],dtype = np.float32)
		# print("tf, shape =", tf, tf.shape )
		while not rospy.is_shutdown():
			if count>self.MARKERS_MAX:
				count = 1
			curr_t = np.array([[self.curr_pose.position.x],[self.curr_pose.position.y],[self.curr_pose.position.z],[1.0]])
			# print("curr_t, shape= ", curr_t,curr_t.shape)
			pose = Pose()
			poseArr.header.frame_id = "/base"
			poseArr.header.stamp = rospy.Time()
			
			trans = np.matmul(tf,curr_t)
			pose.position.x = trans[0]
			pose.position.y = trans[1]
			pose.position.z = trans[2]

			pose.orientation.w = 1.0
			pose.orientation.x = 0.0
			pose.orientation.y = 0.0
			pose.orientation.z = 0.0

			if(len(poseArr.poses)>self.MARKERS_MAX):
				poseArr.poses.pop(0)

			poseArr.poses.append(pose)
			self.pose_pub.publish(poseArr)

			# print count
			count+=1
			rospy.sleep(0.05)



		


def main():
	
	rospy.init_node('traj_plot')
	obj = traj_plot()
	obj.plot()

if __name__ == '__main__':
	main()