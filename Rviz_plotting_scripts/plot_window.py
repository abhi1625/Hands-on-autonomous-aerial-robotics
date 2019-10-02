#!/usr/bin/env python 
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import ColorRGBA
import tf.transformations

class traj_plot:

	def __init__(self):
		self.topic = 'viz_traj'
		# self.pose_topic = 'pose_traj'
		self.marker_pub = rospy.Publisher(self.topic, MarkerArray, queue_size = 50)
		self.MARKERS_MAX = 5
		self.marker_array = MarkerArray()
	
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
		
		

	def plot(self):
		count = 1
		rate = rospy.Rate(10)
		# for ind in range(2):
		# 	if ind==0:
		# 		marker.pose.position.x = -2
		# 		marker.pose.position.y = 2
		# 		marker.pose.position.z = 0
		# 		quaternion = tf.transformations.quaternion_from_euler(0, 1.57,0)
		# 		marker.pose.orientation.w = quaternion[0]
		# 		marker.pose.orientation.x = quaternion[1]
		# 		marker.pose.orientation.y = quaternion[2]
		# 		marker.pose.orientation.z = quaternion[3]
		# 		marker.scale.x = 0.1
		# 		marker.scale.y = 0.1
		# 		marker.scale.z = 0.5

		# 		marker_array.markers.append(marker)
		# 	elif ind ==1:
		# 		marker.pose.position.x = 2
		# 		marker.pose.position.y = 2
		# 		marker.pose.position.z = 0
		# 		quaternion = tf.transformations.quaternion_from_euler(0, 1.57,0)
		# 		marker.pose.orientation.w = quaternion[0]
		# 		marker.pose.orientation.x = quaternion[1]
		# 		marker.pose.orientation.y = quaternion[2]
		# 		marker.pose.orientation.z = quaternion[3]
		# 		marker.scale.x = 0.1
		# 		marker.scale.y = 0.1
		# 		marker.scale.z = 0.5

		# 		marker_array.markers.append(marker)
				

		while not rospy.is_shutdown():
			print(len(self.marker_array.markers))

			bar_length = 1.0  #meter
			pole_length = 1.0 #meter
			stand_length = 0.5 #meter
			self.marker_array.markers = []
			timestamp = rospy.Time.now()
			# left leg
			base_pos = np.array([-bar_length/2,0.0,0.0])
			base_angle = np.array([0.0,1.57,1.57])
			scale = np.array([0.1,0.1,stand_length])
			print("base angle = ", base_angle[0],base_angle[1],base_angle[2])
			self.create_marker(base_pos,base_angle,scale,timestamp,1)

			# right leg
			scale = np.array([0.1,0.1,stand_length])
			pos = np.array([base_pos[0]+1.0,base_pos[1],base_pos[2]])
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
			pos = np.array([[base_pos[0]+1],[base_pos[1]],[base_pos[2]+scale[2]/2]])
			angle = np.array([[base_angle[0]],[base_angle[1]-1.57],[base_angle[2]-1.57]])
			self.create_marker(pos,angle,scale,timestamp,4)

			# upper bar
			scale = np.array([[0.1],[0.1],[bar_length]])
			pos = np.array([base_pos[0]+bar_length/2,base_pos[1],base_pos[2]+pole_length/2])
			angle = np.array([base_angle[0]+1.57,base_angle[1]-1.57,base_pos[2]-1.57])
			self.create_marker(pos,angle,scale,timestamp,5)
			self.marker_pub.publish(self.marker_array)

			# lower bar
			scale = np.array([[0.1],[0.1],[bar_length]])
			pos = np.array([base_pos[0]+bar_length/2,base_pos[1],base_pos[2]+pole_length])
			angle = np.array([base_angle[0]+1.57,base_angle[1]-1.57,base_pos[2]-1.57])
			self.create_marker(pos,angle,scale,timestamp,6)
			self.marker_pub.publish(self.marker_array)

			for i in range(len(self.marker_array.markers)):
				print("zpos of ",i," ",self.marker_array.markers[i].pose.orientation.x)
			# print count
			# count+=1
			rospy.sleep(0.1)



		


def main():
	
	rospy.init_node('traj_plot')
	obj = traj_plot()
	obj.plot()

if __name__ == '__main__':
	main()