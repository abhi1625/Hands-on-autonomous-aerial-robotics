#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import math
import os
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist,Pose
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry


class StereoVO:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub_l = rospy.Subscriber("/duo3d/left/image_rect", Image, self.img_cb_l)
		self.image_sub_r = rospy.Subscriber("/duo3d/right/image_rect", Image, self.img_cb_l)
		self.image_sub_mono = rospy.Subscriber("/image_raw", Image, self.img_cb_mono)
		self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
		self.detect_corner_reset_num = 20
		# self.surf = cv2.xfeatures2d.SURF_create(hessianThreshold = 900)
		self.feature_params = dict( maxCorners = 100,
		               qualityLevel = 0.3,
		               minDistance = 7,
		               blockSize = 7 )

		# Parameters for lucas kanade optical flow
		self.lk_params = dict( winSize  = (15,15),
		                  maxLevel = 2,
		                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
		# Create some random colors
		self.color = np.random.randint(0,255,(100,3))
		self.stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
		self.K_stereo = np.array([  [201.850, 0, 300.229],
									[0, 202.178, 223.933],
									[0, 0, 1]
											], dtype=np.float32)

		self.K_mono = np.array([  [685.6401,    0, 		428.4278],
									[0, 		683.9121,   238.2167],
									[0, 		0,				 1  ]
											], dtype=np.float32)

		self.f_mono = 0.5*(self.K_mono[0,0]+self.K_mono[1,1])   #cm
		
		self.f_stereo = 0.5*(self.K_stereo[0,0]+self.K_stereo[1,1])   #cm
		self.baseline_stereo = 3.0 #cm
		self.features = None
		self.pose = Pose()
		# fig = plt.figure()
		# self.ax = fig.gca(projection ='3d')
		self.h = 460 
		self.w = 800
		self.img_l = None
		self.img_r = None
		self.img_mono = None
		self.mono_l = None
		self.mono_r = None
		self.init_pos = None
		self.final_pos = None
		self.track_ob = trajectory_track()
		self.vel = Twist()
		self.flow_pub = rospy.Publisher('/flow_img', Image, queue_size = 1)
		self.curr_frame_pub = rospy.Publisher('/curr_frame', Image, queue_size=1)
		self.tri_flow = []
	def img_cb_l(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# print('got frame')
		except CvBridgeError as e:
			print(e)
		
		if cv_image is not None:
			self.img_l = cv_image

	def img_cb_r(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# print('got frame')
		except CvBridgeError as e:
			print(e)
		
		if cv_image is not None:
			self.img_r = cv_image

	def img_cb_mono(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# print('got frame')
		except CvBridgeError as e:
			print(e)
		
		if cv_image is not None:
			self.img_mono = cv_image
			# print("img mono shape",self.img_mono.shape)


	def odom_callback(self, data):
		self.pose = data.pose.pose
		# print "pose ", self.pose
	
	def move_right(self, target_pos):
		
		rate = rospy.Rate(10)
		_, = self.track_ob.mission(True,vel,target_pos,1,rate)


	def get_depth(self, img_left, img_right,baseline):
		img_left_gray = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
		img_right_gray = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
		disparity = self.stereo.compute(img_left_gray,img_right_gray)
		disparity = np.float32(disparity)
		disparity[disparity == 0.0] = 0.0001
		# disparity3d = np.dstack((disparity,disparity,disparity))
		# print('disparity max and min',np.amax(disparity),np.amin(disparity), np.mean(np.mean(disparity)))
		print("mean disparity = {}".format(np.mean(np.mean(disparity))))
		depth_map = np.reciprocal(disparity)*baseline*self.f_mono
		print("mean depth = {}".format(np.mean(np.mean(depth_map))))
		# cv2.imshow("disparity",depth_map)
		# cv2.waitKey(70)
		return depth_map

	def compute_error(self,prev_frame,curr_frame,ind):
		h_error = -400
		v_error = -230
		img_left_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
		img_right_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
		hsv = np.zeros_like(prev_frame[:,:,0])
		# hsv = np.zeros_like(prev_frame)
		# hsv[...,1] = 255
		flow = cv2.calcOpticalFlowFarneback(img_left_gray,img_right_gray, None, 0.5, 3, 30, 3, 5, 1.5, 0)
		mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
		# hsv[...,0] = ang*180/np.pi/2
		hsv = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
		flow_img = np.uint8(hsv)
		# self.tri_flow.append(flow_img)
		# if len(self.tri_flow) >3:
		# 	del self.tri_flow[0]
		# mean_flow = np.mean(np.array(self.tri_flow), axis=0)
		# print("mean_flow_shape: ",mean_flow.shape)
		# mean_flow = cv2.normalize(mean_flow,None,0,255,cv2.NORM_MINMAX)
		# mean_flow = np.uint8(mean_flow)
		flow_msg = self.bridge.cv2_to_imgmsg(flow_img, "mono8")
		curr_msg = self.bridge.cv2_to_imgmsg(curr_frame, "bgr8")
		self.flow_pub.publish(flow_msg)
		self.curr_frame_pub.publish(curr_msg)
		keypoints = self.get_wall_center(flow_img)
		if len(keypoints) >=1:
			h_error = (self.w/2) - float(keypoints[0].pt[0])
			v_error = (self.h/2) - float(keypoints[0].pt[1])
		return h_error, v_error

	def detect_blobs(self,im):
		# Setup SimpleBlobDetector parameters.
		params = cv2.SimpleBlobDetector_Params()
			
		# Change thresholds
		params.minThreshold = 10
		params.maxThreshold = 255
		params.blobColor = 255
		params.filterByColor = True
			
		# Filter by Area.
		params.filterByArea = True
		params.minArea = 2500
		params.maxArea = 10000000000000
			
		# # Filter by Circularity
		params.filterByCircularity = False
		# params.minCircularity = 0.1
			
		# # Filter by Convexity
		params.filterByConvexity = False
		# params.minConvexity = 0.9
			
		# # Filter by Inertia
		params.filterByInertia = False
		# params.minInertiaRatio = 0.1
			
		# Create a detector with the parameters
		ver = (cv2.__version__).split('.')
		if int(ver[0]) < 3 :
			detector = cv2.SimpleBlobDetector(params)
		else : 
			detector = cv2.SimpleBlobDetector_create(params)

		keypoints = detector.detect(im)
		# if keypoints:
		#     print("keypoints = ",keypoints[0].pt)

		# Draw detected blobs as red circles.
		# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
		# the size of the circle corresponds to the size of blob

		# im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

		# # Show blobs
		# cv2.imshow("Keypoints", im_with_keypoints)
		# cv2.waitKey(1)
		# cv2.destroyAllWindows()
		return keypoints

	def get_wall_center(self,cv_image):
		cv_image = np.dstack((cv_image, cv_image, cv_image))
		kernel = np.ones(30)
		h = cv_image.shape[0]
		w = cv_image.shape[1]
		closing = cv2.morphologyEx(cv_image, cv2.MORPH_CLOSE, kernel)
		thresh = np.mean(closing[:])
		closing[closing < thresh] = 0
		closing_resize = cv2.resize(closing, (w/4, h/4))
		Z = closing_resize.reshape((-1,3))

		# convert to np.float32
		Z = np.float32(Z)

		# define criteria, number of clusters(K) and apply kmeans()
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
		K = 4
		ret,label,center=cv2.kmeans(Z,K,None,criteria,5,cv2.KMEANS_RANDOM_CENTERS)
		print(np.amax(label), np.amin(label), center.shape)
		# Now convert back into uint8, and make original image
		center = np.uint8(center)
		res = center[label.flatten()]
		max_center = np.amax(res)
		print(max_center)
		res[res<max_center] = 0
		res[res>=max_center] = 255
		res2 = res.reshape((h/4,w/4,3))
		closing = cv2.resize(res2,(w,h))
		closing = np.uint8(closing)
		keypoints = self.detect_blobs(closing)
		print("length",len(keypoints))
		if len(keypoints) >= 1:
			center_x = np.mean(np.array(keypoints[0].pt[0]))
			center_y = np.mean(np.array(keypoints[0].pt[1]))
			cv2.circle(cv_image,(int(center_x),int(center_y)),int(keypoints[0].size/2),(255,0,255),2)
		#cv2.imshow("test_img",cv_image)
		#cv2.waitKey(1)
		return keypoints

	def autonomy_up(self):
		# check mono image
		# assuming quad is holding height
		print("inside run pipeline")
		h=1.0
		target_pos_init = np.array([0.0,0.0,h])
		mission_hold_pos = True
		mission_move_right = False
		mission_land = False
		curr_frame = None
		prev_frame = None
		self.track_ob.takeoff()
		rospy.sleep(3)
		rate = rospy.Rate(10)
		ind = 0
		allowance = 10
		mission_h_align = True
		mission_v_align = False
		final_flag = True
		go_up = True
		go_down = False
		go_right = False
		go_left = False
		while(not rospy.is_shutdown()):
			curr_frame = self.img_mono
			# curr_frame = cv2.resize(curr_frame, (800/2, 460/2))
			curr_frame = cv2.medianBlur(curr_frame, 5)
			if prev_frame is not None:
				if mission_hold_pos == True:
					mission_hold_pos, mission_move_right = self.track_ob.mission(mission_hold_pos, self.vel, target_pos_init, 1, rate)
					# rospy.sleep(2)
					self.init_pos = self.pose
					prev_frame = self.img_mono 
					# filename = "initial_frame_"+str(ind)+".jpg" 
					# cv2.imwrite(filename, prev_frame)
					print("captured 1st frame")
					mission_hold_pos = False
				if mission_h_align == True:
					if go_up== True:
						self.vel.linear.z = 0.2
						self.track_ob.bebop_vel_pub.publish(self.vel)	
					else :
						self.vel.linear.z = 0.0
					right_frame = self.img_mono.copy()
					h_error, v_error = self.compute_error(prev_frame, right_frame,ind)
					if -allowance < h_error < allowance:
						self.vel.linear.y = 0.0
						self.track_ob.bebop_vel_pub.publish(self.vel)
						mission_h_align = False
						mission_v_align = True
						go_right = True
					else:
						self.vel.linear.y = 0.001*h_error
						self.track_ob.bebop_vel_pub.publish(self.vel)
					if self.track_ob.current_state[2] >= 1.5:
						go_up = False
						go_down = True
					# elif go_down == True:
					# 	self.vel.linear.z = -0.2
					# 	self.track_ob.bebop_vel_pub.publish(self.vel)	
					# 	right_frame = self.img_mono.copy()
					# 	h_error, v_error = self.compute_error(prev_frame, right_frame,ind)
					# 	if -allowance < h_error < allowance:
					# 		self.vel.linear.y = 0.0
					# 		self.track_ob.bebop_vel_pub.publish(self.vel)
					# 		mission_h_align = False
					# 		missioin_v_align = True
					# 		go_right = True
					# 		print("######################### mission_h_align complete")
					# 	else:
					# 		self.vel.linear.y = 0.001*h_error
					# 		self.track_ob.bebop_vel_pub.publish(self.vel)
					# 	if self.track_ob.current_state[2] < 1.0:
					# 		go_up = True
					# 		go_down = False
				# # elif mission_v_align == True:
				# # 	if go_right== True:
				# # 		self.vel.linear.y = -0.1
				# # 		self.track_ob.bebop_vel_pub.publish(self.vel)	
				# # 		right_frame = self.img_mono.copy()
				# # 		h_error, v_error = self.compute_error(prev_frame, right_frame,ind)
				# # 		if -allowance < v_error < allowance:
				# # 			self.vel.linear.z = 0.0
				# # 			self.track_ob.bebop_vel_pub.publish(self.vel)
				# # 			mission_v_align = False
				# # 			# go_right = True
				# # 		else:
				# # 			self.vel.linear.z = 0.001*v_error
				# # 			self.track_ob.bebop_vel_pub.publish(self.vel)
				# # 			go_right = False
				# # 			go_left = True
				# 	elif go_left == True:
				# 		self.vel.linear.y = 0.1
				# 		self.track_ob.bebop_vel_pub.publish(self.vel)	
				# 		right_frame = self.img_mono.copy()
				# 		h_error, v_error = self.compute_error(prev_frame, right_frame,ind)
				# 		if -allowance < v_error < allowance:
				# 			self.vel.linear.z = 0.0
				# 			self.track_ob.bebop_vel_pub.publish(self.vel)	
				# 			mission_v_align = False
				# 			# go_right = True
				# 		else:
				# 			self.vel.linear.z = 0.001*v_error
				# 			self.track_ob.bebop_vel_pub.publish(self.vel)	
				# 			go_right = True
				# 			go_left = False
				else:
					h_error, v_error = self.compute_error(prev_frame, curr_frame,ind)
					# if ((self.track_ob.current_state[2] > 1.5) and (final_flag == True)):
					if ((v_error) > 0 and (final_flag == True)):
						target_pos_final = np.array([self.track_ob.current_state[0],self.track_ob.current_state[1], 1.0])
						go_ahead = np.array([5.0,self.track_ob.current_state[1], 1.0])
						_,_ = self.track_ob.mission(mission_v_align, self.vel, target_pos_final, 1, rate)
						_,_ = self.track_ob.mission(mission_h_align, self.vel, go_ahead, 2, rate)
						self.track_ob.land()
						rospy.sleep(5)
						# final_flag = False
					elif ((v_error) < 0 and (final_flag == True)):
					# elif ((self.track_ob.current_state[2] < 1.5) and (final_flag == True)) :
						target_pos_final = np.array([self.track_ob.current_state[0],self.track_ob.current_state[1], 2.25])
						go_ahead = np.array([5.0,self.track_ob.current_state[1], 2.25])
						_,_ = self.track_ob.mission(mission_v_align, self.vel, target_pos_final, 3, rate)
						_,_ = self.track_ob.mission(mission_h_align, self.vel, go_ahead, 4, rate)
						self.track_ob.land()
						rospy.sleep(5)

			ind+=1
			# if ind%2 == 0:
			prev_frame=curr_frame			   

def main():
	rospy.init_node('wall_detection',anonymous=True)
	StereoVO_obj = StereoVO()
	# print("executed main")
	StereoVO_obj.autonomy_up()
	# plt.show()
if __name__ == '__main__':
    main()

