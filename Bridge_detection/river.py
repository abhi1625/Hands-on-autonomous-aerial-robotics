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
# from ../trajectory.mission_script import trajectory_track
# sys.path.insert(1, '../trajectory')
# from mission_script import trajectory_track


class StereoVO:
	def __init__(self):
		self.bridge = CvBridge()
		# self.image_sub_l = rospy.Subscriber("/duo3d/left/image_rect", Image, self.img_cb_l)
		# self.image_sub_r = rospy.Subscriber("/duo3d/right/image_rect", Image, self.img_cb_l)
		self.image_sub_mono = rospy.Subscriber("/image_raw", Image, self.img_cb_mono)
		# self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback)
        	self.state_pub = rospy.Subscriber('/current_state', Pose, self.odom_callback)
		self.blobs_pub = rospy.Publisher("/blobs_img", Image, queue_size = 1)
		self.bridge_pub = rospy.Publisher("/bridge_img", Image, queue_size = 1)
		self.pose_pub = rospy.Publisher("/relative_pose", Pose, queue_size = 1)

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
		# self.stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
		# self.K_stereo = np.array([  [201.850, 0, 300.229],
		# 							[0, 202.178, 223.933],
		# 							[0, 0, 1]
		# 									], dtype=np.float32)

		self.K_mono = np.array([  [685.6401,    0, 		428.4278],
									[0, 		683.9121,   238.2167],
									[0, 		0,				 1  ]
											], dtype=np.float32)

		# self.f_mono = 0.5*(self.K_mono[0,0]+self.K_mono[1,1])   #cm
		
		#self.f_stereo = 0.5*(self.K_stereo[0,0]+self.K_stereo[1,1])   #cm
		# self.baseline_stereo = 3.0 #cm
		# self.features = None
		self.pose = Pose()
		self.relative_pose = Pose()
		# fig = plt.figure()
		# self.ax = fig.gca(projection ='3d')
		self.h = 720 
		self.w = 1280
		# self.img_l = None
		# self.img_r = None
		self.img_mono = None
		# self.mono_l = None
		# self.mono_r = None
		# self.init_pos = None
		# self.final_pos = None
		# self.track_ob = trajectory_track()
		self.vel = Twist()
		self.take_off_flag = False
		self.error = None
		# self.prev_error = 0.0
		# self.max_arr_len = 10
		# self.vel = Twist()
	

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
		self.pose = data
		# print "pose ", self.pose
	
	# def move_right(self, target_pos):
		
	# 	rate = rospy.Rate(10)
	# 	_, = self.track_ob.mission(True,vel,target_pos,1,rate)


	# def get_depth(self, img_left, img_right,baseline):
	# 	img_left_gray = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
	# 	img_right_gray = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
	# 	disparity = self.stereo.compute(img_left_gray,img_right_gray)
	# 	disparity = np.float32(disparity)
	# 	disparity[disparity == 0.0] = 0.0001
	# 	# disparity3d = np.dstack((disparity,disparity,disparity))
	# 	# print('disparity max and min',np.amax(disparity),np.amin(disparity), np.mean(np.mean(disparity)))
	# 	print("mean disparity = {}".format(np.mean(np.mean(disparity))))
	# 	depth_map = np.reciprocal(disparity)*baseline*self.f_mono
	# 	print("mean depth = {}".format(np.mean(np.mean(depth_map))))
	# 	# cv2.imshow("disparity",depth_map)
	# 	# cv2.waitKey(70)
	# 	return depth_map

	# def compute_error(self,prev_frame,curr_frame,ind):
	# 	img_left_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
	# 	img_right_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
	# 	# hsv = np.zeros_like(prev_frame[:,:,0])
	# 	hsv = np.zeros_like(prev_frame)
	# 	hsv[...,1] = 255
	# 	flow = cv2.calcOpticalFlowFarneback(img_left_gray,img_right_gray, None, 0.5, 3, 20, 3, 5, 1.5, 0)
	# 	mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
	# 	hsv[...,0] = ang*180/np.pi/2
	# 	hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
	# 	hsv = np.uint8(hsv)
	# 	#median_hsv = 1.2*np.median(hsv[:])

	# 	#hsv[hsv < median_hsv] = 0 
	# 	rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
	# 	# cluster_img = np.dstack((hsv[:,0],hsv[:,2]))
	# 	# rgb = cv2.GaussianBlur(rgb, (5,5),0)
	# 	kernel = np.ones(11)
	# 	# hsv = cv2.morphologyEx(hsv,cv2.MORPH_CLOSE, kernel)
	# 	# cv2.imshow('flow',hsv)
	# 	vel = Twist()
	# 	filename = "flow_"+str(ind)+".jpg" 
	# 	print("filename flow = ",filename)
	# 	cv2.imwrite(filename,rgb)
	# 	#cv2.imshow('flow',rgb)
	# 	#cv2.waitKey(70)
	# 	# cv2.destroyAllWindows()
	# 	#hsv[hsv[:]== 0.0] = 0.000000e1
	# 	return 0.0

	
	def segment_river(self,cv_image):
		# river params dark night
		# thresh_r_min=87
		# thresh_g_min=81
		# thresh_b_min=133
		# thresh_r_max=155
		# thresh_g_max=205
		# thresh_b_max=255

		# # river params day light 100
		thresh_r_min=119
		thresh_g_min=138
		thresh_b_min=178
		thresh_r_max=152
		thresh_g_max=209
		thresh_b_max=255

		# river params bright day
		#thresh_r_min=16
		#thresh_g_min=72
		#thresh_b_min=0
		#thresh_r_max=110
		#thresh_g_max=248
		#thresh_b_max=248

		cv_image[cv_image[:,:,0] < thresh_b_min]=0
		cv_image[cv_image[:,:,1] < thresh_g_min]=0
		cv_image[cv_image[:,:,2] < thresh_r_min]=0

		cv_image[cv_image[:,:,0] > thresh_b_max]=0
		cv_image[cv_image[:,:,1] > thresh_g_max]=0
		cv_image[cv_image[:,:,2] > thresh_r_max]=0
		return cv_image
			   


	def detect_blobs(self,im):
		# Setup SimpleBlobDetector parameters.
		params = cv2.SimpleBlobDetector_Params()
		 
		# Change thresholds
		params.minThreshold = 200
		params.maxThreshold = 256
		params.blobColor = 255
		 
		# Filter by Area.
		params.filterByArea = True
		params.minArea = 1500
		params.maxArea = 1000000
		 
		# Filter by Circularity
		params.filterByCircularity = False
		params.minCircularity = 0.1
		 
		# Filter by Convexity
		params.filterByConvexity = False
		params.minConvexity = 0.87
		 
		# Filter by Inertia
		params.filterByInertia = True
		params.minInertiaRatio = 0.001
		 
		# Create a detector with the parameters
		ver = (cv2.__version__).split('.')
		if int(ver[0]) < 3 :
		    detector = cv2.SimpleBlobDetector(params)
		else : 
		    detector = cv2.SimpleBlobDetector_create(params)

		keypoints = detector.detect(im)
		# if keypoints:
		# 	print("keypoints = ",keypoints[0].pt)

		# Draw detected blobs as red circles.
		# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
		# the size of the circle corresponds to the size of blob

		im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

		# Show blobs
		# cv2.imshow("Keypoints", im_with_keypoints)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		return keypoints

	# def detect_bridge_first(self,im):
	# 	# Setup SimpleBlobDetector parameters.
	# 	params = cv2.SimpleBlobDetector_Params()
		 
	# 	# Change thresholds
	# 	params.minThreshold = 200
	# 	params.maxThreshold = 256
	# 	params.blobColor = 255
		 
	# 	# Filter by Area.
	# 	params.filterByArea = True
	# 	params.minArea = 500
	# 	params.maxArea = 2500
		 
	# 	# Filter by Circularity
	# 	params.filterByCircularity = False
	# 	params.minCircularity = 0.1
		 
	# 	# Filter by Convexity
	# 	params.filterByConvexity = False
	# 	params.minConvexity = 0.87
		 
	# 	# Filter by Inertia
	# 	params.filterByInertia = True
	# 	params.minInertiaRatio = 0.001
		 
	# 	# Create a detector with the parameters
	# 	ver = (cv2.__version__).split('.')
	# 	if int(ver[0]) < 3 :
	# 	    detector = cv2.SimpleBlobDetector(params)
	# 	else : 
	# 	    detector = cv2.SimpleBlobDetector_create(params)

	# 	keypoints = detector.detect(im)
	# 	return keypoints

	def detect_bridge(self,im):
		# Setup SimpleBlobDetector parameters.
		params = cv2.SimpleBlobDetector_Params()
		 
		# Change thresholds
		params.minThreshold = 200
		params.maxThreshold = 256
		params.blobColor = 255
		 
		# Filter by Area.
		params.filterByArea = True
		params.minArea = 500
		params.maxArea = 2500
		 
		# Filter by Circularity
		params.filterByCircularity = False
		params.minCircularity = 0.1
		 
		# Filter by Convexity
		params.filterByConvexity = False
		params.minConvexity = 0.87
		 
		# Filter by Inertia
		params.filterByInertia = True
		params.minInertiaRatio = 0.001
		 
		# Create a detector with the parameters
		ver = (cv2.__version__).split('.')
		if int(ver[0]) < 3 :
		    detector = cv2.SimpleBlobDetector(params)
		else : 
		    detector = cv2.SimpleBlobDetector_create(params)

		keypoints = detector.detect(im)
		return keypoints

	def transform_img(self,img):
		per = 0.6
		# pts_src = np.array([[0.0, 0.0],[0.0, self.w],[self.h, self.w],[self.h, 0]],dtype = np.float32)
		# pts_dst = np.array([[self.h*per, 0],[0, self.w*per],[self.h, self.w],[self.h, 0]],dtype = np.float32)
		# print("src and dst = ",pts_src,pts_dst)
		# h, status = cv2.findHomography(pts_src, pts_dst)		 
		# im_dst = cv2.warpPerspective(img, h, (img.shape[1],img.shape[0]))
		crop = img[int(self.h*per):,:]
		crop = cv2.resize(crop,(self.w,self.h),interpolation = cv2.INTER_AREA)
		
		return crop

	def bridge_detect(self, curr_frame):
		if curr_frame is not None:
			print("inside curr frame")
			img_center_x = float(curr_frame.shape[1]/2)
			img_center_y = float(curr_frame.shape[0]/2)
			# print("image center = ", img_center_x)
			curr_frame = self.transform_img(curr_frame)
			# cv2.imshow('after homography',curr_frame)
			# cv2.waitKey(10)
			river_seg = self.segment_river(curr_frame)
			# cv2.imshow('thresh img',curr_frame)
			# cv2.waitKey(1)
			river_mask = np.float32(cv2.cvtColor(river_seg,cv2.COLOR_BGR2GRAY))
			# river_mask[:int(img_center_y),:] = 0.0
			river_mask = np.uint8(river_mask)
			kernel = (7,7)
			# river_mask = cv2.GaussianBlur(river_mask,kernel,0)
			river_mask = cv2.medianBlur(river_mask,5,0)

			# cv2.imshow('median ',river_mask)
			# cv2.waitKey(1)

			kernel = np.ones((7,7),np.uint8)
			river_mask = cv2.morphologyEx(river_mask, cv2.MORPH_OPEN, kernel)
			kernel = np.ones((20,20),np.uint8)
			river_mask = cv2.morphologyEx(river_mask, cv2.MORPH_CLOSE, kernel)

			# cv2.imshow('closing ',river_mask)
			# cv2.waitKey(10)
			# kernel = np.ones((5,5),np.uint8)
			# river_mask = cv2.dilate(river_mask,kernel,iterations = 1)
			river_mask[river_mask[:]>0.0] = 255.0
			#river_mask[river_mask[:]<] = 255.0

			river_mask = np.uint8(river_mask)	
			#cv2.imshow('mask ',river_mask)
			#cv2.waitKey(1)
			# cv2.destroyAllWindows()
			river_mask_3d = np.dstack((river_mask,river_mask,river_mask))
			
			keypoints_first = self.detect_bridge(river_mask)
			if keypoints_first:
				self.error = img_center_x - keypoints_first[0].pt[0]
				status = True
				return status, self.error
				print("first frame bridge detect")

			else:
				self.error = -400.0

				keypoints = self.detect_blobs(river_mask)
				im_with_keypoints = cv2.drawKeypoints(river_mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

				# Show blobs
				#cv2.imshow("Keypoints", im_with_keypoints)
				#cv2.waitKey(1)

				if keypoints:
					# for i in range(len(keypoints)):
						# print("response of"+str(i)+"keypoint",keypoints[i].response)
					num_keypoints = len(keypoints)
					if num_keypoints == 1:
						
						blob_img= cv2.circle(river_mask_3d,(int(keypoints[0].pt[0]),int(keypoints[0].pt[1])),int(keypoints[0].size),(0,255,0),4)
						pub_img_blob = self.bridge.cv2_to_imgmsg(blob_img)
						self.blobs_pub.publish(pub_img_blob)
						if(self.take_off_flag == False):
							status = True
							self.take_off_flag == True
							self.error = - img_center_x
							# self.prev_error = self.error
							# self.error.append(- img_center_x)
							# return status, np.mean(np.array(self.error))
							return status, self.error

						else:
							# if len(self.error)>self.max_arr_len:
							# 	self.prev_error = np.mean(np.array(self.error))
							# 	self.error.pop(0)
							# 	self.error.append(-img_center_x)
							# else:
							# 	self.prev_error = np.mean(np.array(self.error))
							# 	self.error.append(-img_center_x)
							# 	status = False
							# print("keypoint x = ", keypoints[0].pt[0])
							# return status, np.mean(np.array(self.error))
							self.error = - img_center_x
							status = True
							return status, self.error

					if num_keypoints >= 2:
						# w0 = keypoints[0].size
						# w1 = keypoints[1].size

						# print("w0 and w1", w0,w1)
						# print("x0 y0 = ",x0,y0)
						xerr = int(((keypoints[1].pt[0] + keypoints[0].pt[0] )/2))
						yerr = int(((keypoints[1].pt[1] + keypoints[0].pt[1] )/2))
						# bridge_center= cv2.circle(river_mask_3d,(xerr,yerr),10,(255,0,0),-1)
						bridge_center= cv2.circle(river_mask_3d,(int(keypoints[0].pt[0]),int(keypoints[0].pt[1])),int(keypoints[0].size),(0,0,0),-1)
						bridge_center= cv2.circle(river_mask_3d,(int(keypoints[1].pt[0]),int(keypoints[1].pt[1])),int(keypoints[1].size),(0,0,0),-1)
						# cv2.i11mshow("bridge_center",bridge_center)
						self.blobs_pub.publish()
						# cv2.waitKey(0)
						# cv2.destroyAllWindows()
						center_keypoints = self.detect_bridge(bridge_center)
						if center_keypoints:
							# print("response",center_keypoints[0].pt)
							x_bridge = center_keypoints[0].pt[0]
							y_bridge = center_keypoints[0].pt[1]
							cv2.circle(bridge_center,(int(x_bridge),int(y_bridge)),int(center_keypoints[0].size),(0,234,32),3)
							#cv2.imshow("TEST	",bridge_center)
							pub_img = self.bridge.cv2_to_imgmsg(bridge_center)
							self.bridge_pub.publish(pub_img)
							#cv2.waitKey(1)
							# cv2.destroyAllWindows()

							self.error = img_center_x - x_bridge
							# if len(self.error)>self.max_arr_len:
							# 	self.prev_error = np.mean(np.array(self.error))
							# 	self.error.pop(0)
							# 	self.error.append(new_error)
							# else:
							# 	self.prev_error = np.mean(np.array(self.error))
							# 	self.error.append(new_error)

							#print("error = ", np.mean(np.array(self.error)))
							status = True
							return status, self.error
							
						else:
							print("bridge not detected")
							status = False
							return status, 0.0
		status = False
		return status, 0.0

	def run_pipeline(self):
		# while(not rospy.is_shutdown()):
		curr_frame = self.img_mono

		status, error = self.bridge_detect(curr_frame)
		curr_h = self.pose.position.z
		if (status == True):

			error = error/float(self.w)
			self.relative_pose.position.x = 0.0
			self.relative_pose.position.y = error
			self.relative_pose.position.z = 0.0
			if (curr_h>0.4):
			    self.relative_pose.position.z = -0.03
			self.relative_pose.orientation.x = 0.0
			self.relative_pose.orientation.y = 0.0
			self.relative_pose.orientation.z = -error*2.0
			self.pose_pub.publish(self.relative_pose)
			# self.vel.linear.y = error
			# self.vel.linear.x = 0.0
			# self.vel.linear.z= 0.0
			# self.track_ob.bebop_vel_pub.publish(self.vel)
		elif (status == False):
			if(curr_h>0.4):
				self.relative_pose.position.x = - 0.03
				self.relative_pose.position.y = 0.0
				self.relative_pose.position.z = - 0.03
			else:
				self.relative_pose.position.x = - 0.03
				self.relative_pose.position.y = 0.0
				self.relative_pose.position.z = 0.0
			
		self.pose_pub.publish(self.relative_pose)
		print("status {} and error {} ".format(status, error))

# def main():
# 	rospy.init_node('river_detection',anonymous=True)
# 	StereoVO_obj = StereoVO()
# 	# print("executed main")
# 	StereoVO_obj.run_pipeline()
# 	# plt.show()
# if __name__ == '__main__':
#     main()

