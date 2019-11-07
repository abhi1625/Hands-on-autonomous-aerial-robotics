#!/usr/bin/env python
try:
		sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages/')
except:
		pass

import cv2
import numpy as np
import math
import copy
import os
import rospy
from std_msgs.msg import String,Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist,Pose
from cv_bridge import CvBridge, CvBridgeError

class tatti:
	def __init__(self):
		self.quad_pose = Pose()
		self.target = Pose()
		self.quad = rospy.Subscriber('/quad_pose', Pose, self.quad_cb)
		self.target_sub = rospy.Subscriber('/target_pose_inertial', Pose, self.target_cb)

	def quad_cb(self,data):
		self.quad_pose = data

	def target_cb(self,data):
		self.target = data

cap = cv2.VideoCapture('/home/pratique/Downloads/Bullseye.mp4')
ret, frame = cap.read() 
height, width, layers = frame.shape
out = cv2.VideoWriter('/home/pratique/Downloads/test.avi', 0, fps=60,frameSize=(width,height))
i=0
rospy.init_node('taatti', anonymous=True)
lendi = tatti()
rate = rospy.Rate(60)
while(cap.isOpened() and i<14*60):
	ret, frame = cap.read()
	frame[:160,:400,:] = 0
	frame[:160,1520:,:] = 0
	cv2.putText(frame,'x_quad: '+str(round(lendi.quad_pose.position.x,4)), (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0),1, lineType=cv2.LINE_AA)
	cv2.putText(frame,'y_quad: '+str(round(lendi.quad_pose.position.y,4)), (20,80), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0),1, lineType=cv2.LINE_AA)
	cv2.putText(frame,'z_quad: '+str(round(lendi.quad_pose.position.z,4)), (20,120), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0),1, lineType=cv2.LINE_AA)

	cv2.putText(frame,'x_target: '+str(round(lendi.target.position.x,4)), (1530,40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0),1, lineType=cv2.LINE_AA)
	cv2.putText(frame,'y_target: '+str(round(lendi.target.position.y,4)), (1530,80), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0),1, lineType=cv2.LINE_AA)
	cv2.putText(frame,'z_target: '+str(round(lendi.target.position.z,4)), (1530,120), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 0),1, lineType=cv2.LINE_AA)
	# cv2.imshow("frame",frame)
	# cv2.waitKey(1)
	out.write(frame)
	rate.sleep()
	print(i)
	i+=1
cap.release()
out.release()
cv2.destroyAllWindows()
# video.release()