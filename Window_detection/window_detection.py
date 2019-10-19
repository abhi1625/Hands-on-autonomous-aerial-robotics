#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
try:
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages/')
	sys.path.remove('/opt/ros/kinetic/share/opencv3/')
except:
	pass

import cv2
import numpy as np
import math
import copy
from GMM.test_data import *

class video_stream:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_raw", Image, self.img_callback)
		self.window_detection = Window_detection()
		weights_path = '/home/abhinav/Gits/drone-course/Window_detection/GMM/training_params/window_weights_4.npy'
		params_path = '/home/abhinav/Gits/drone-course/Window_detection/GMM/training_params/gaussian_params_4.npy'
		self.n, self.K, self.weights, self.params = loadparamsGMM(weights_path, params_path)

	def img_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# print('got frame')
		except CvBridgeError as e:
			print(e)
		
		if cv_image is not None:
			processed_img = preprocess_img(cv_image)

		# run GMM inference to generate mask
			mask = test_combined(processed_img,self.K,self.n,self.weights, self.params,(0,255,0))
			self.window_detection.detection_hough_lines(processed_img,mask)
			#self.windo1w_detection.Detection_using_threshold(processed_img)
	
class Window_detection:
	def __init__(self):
		# self.data_path = '/home/prgumd/Desktop/pink_window.mp4'
		self.data_path = '/home/abhinav/drone_course_data/window_detection/pink_window.mp4'
		self.image_path = './GMM/test_image.jpg'
		self.original_image = '/home/abhinav/drone_course_data/Window_detection/GMM/data/GMM_1/frame0200.jpg'
		self.image_pub = rospy.Publisher("/gmm_img", Image, queue_size=2)
		self.bridge = CvBridge()

		################################################
		#moving mean filter
		self.center = np.array([0,0])
		self.centerpoint = np.array([0,0])
		self.count = 0
		self.top_left_corner = np.array([0,0])
		self.top_right_corner = np.array([0,0])
		self.bottom_left_corner = np.array([0,0])
		self.bottom_right_corner = np.array([0,0])
		self.top_left = np.array([0,0])
		self.top_right = np.array([0,0])
		self.bottom_left = np.array([0,0])
		self.bottom_right = np.array([0,0])



	# def rotate_img_90_deg(self,img):
	# 	h,w = img.shape[:2]
	# 	center = (w/2,h/2)
	# 	M = cv2.getRotationMatrix2D(center, -90, 0.5)
	# 	M[0,-1] = M[0,-1] - ((w-h/2)/2)
	# 	M[-1,-1] = M[-1,-1] - ((h-w/2)/2)
	# 	warped_img = cv2.warpAffine(copy.deepcopy(img), M, (h/2,w/2))
	# 	return warped_img

	def get_all_corners(self,img,img_orig,thresh = 1):
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# gray = cv2.GaussianBlur(gray,(7,7),cv2.BORDER_DEFAULT)
		gray = cv2.medianBlur(gray,5)

		# edges = cv2.Canny(gray,50,150,apertureSize = 3)
		edges = gray
		# print("canny type", np.shape(edges))
		cv2.imshow('image',edges)
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(31,31))
		# closing to fill unwanted small gaps
		closing = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
		
		closing = cv2.GaussianBlur(closing, (7,7), cv2.BORDER_DEFAULT)
		corners = cv2.goodFeaturesToTrack(closing, 111, 0.1, 30)

		return np.squeeze(corners)

	def get_outer_window_corner_points(self, corner_pts, img):
		distances = corner_pts[:,0]**2 + corner_pts[:,1]**2
		h,w = img.shape[:2]
		print("corner_pts = ",corner_pts)
		left_pt = np.argmin(distances)
		left_top_corner = corner_pts[left_pt,:]
		# print("")

		distances = (corner_pts[:,0]-w)**2 + (corner_pts[:,1])**2
		right_top_corner = corner_pts[np.argmin(distances),:]

		bottom_most_pt = np.argmax(corner_pts[:,1])
		# print("bottom_most_pt = ", corner_pts[bottom_most_pt,:])
		# corner_pts = np.delete(corner_pts,bottom_most_pt,0)


		distances = (corner_pts[:,0]-w)**2 + (corner_pts[:,1]-h)**2
		right_bottom_corner = corner_pts[np.argmin(distances),:]

		distances = corner_pts[:,0]**2 + (corner_pts[:,1]-h)**2
		left_bottom_corner = corner_pts[np.argmin(distances),:]

		imgPoints = np.array([[left_top_corner],
							  [right_top_corner],
							  [right_bottom_corner],
							  [left_bottom_corner]],dtype=np.int32)
		imgPoints = np.squeeze(imgPoints)

		return imgPoints

	def pnp(self, imgPoints,img):
		h,w = img.shape[:2]
		# World coordinates using window measurement in world
		objPoints = np.array([[0,0,0],
							 [84,0,0],
							 [81,43,0],
							 [0,43,0]], dtype=np.float64)

		# Camera K matrix(intrinsic params)
		camMatrix = np.array([[685.6401304538527, 0 , 428.4278693789007],[0, 683.912176820224, 238.21676543821124],[0,0,1]],dtype=np.float32)

		#distortion coefficients 
		distCoeffs = np.array([0,0,0,0,0],dtype=np.float64)

		_, rotVec, transVec = cv2.solvePnP(objPoints,imgPoints, camMatrix, distCoeffs)

		print("rotation_vec: ",rotVec)
		print("trans_vec: ", transVec)

		# Verification by reporjecting points using rotation and 
		# translation computed above
		reprojPoints,_ = cv2.projectPoints(objPoints, rotVec, transVec, camMatrix, distCoeffs)
		reprojPoints = np.squeeze(reprojPoints)
		# print(reprojPoints.shape)

		for i in range(4):
		        pt = reprojPoints[i]
		        imgpt = imgPoints[i]
		        cv2.circle(img, (int(pt[0]), int(pt[1])),5,[255,0,0],-1)
		        # cv2.circle(img, (int(imgpt[0]),int(imgpt[1])),5,[0,255,0],-1)
		# cv2.imshow('image',img)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()

		return rotVec,transVec


	def Detection_using_threshold(self,img):
		#vidcap = cv2.VideoCapture(self.data_path)
		#count = 0
		#while count <250:
		#	success,img = vidcap.read()
		# while count<1:
		# if((count<250) and(count%40 ==0)):
			# print("image dim = ",np.shape(img))
			# print('Read frame # ', count+1)
		
		# img = self.rotate_img_90_deg(img)
		print("mean r = ", np.mean(np.mean(img[:,:,0])))
		print("mean g = ", np.mean(np.mean(img[:,:,1])))
		print("mean b = ", np.mean(np.mean(img[:,:,2])))

		img_pink = np.logical_and(np.logical_and(img[:,:,0]<10,img[:,:,0]>130,img[:,:,0]<200),img[:,:,1]<150)
		
		img_pink = np.dstack((img_pink,img_pink,img_pink))
		img_pink = img_pink*img

		corner_pts = self.get_all_corners(img_pink,img)
		imgPoints = self.get_outer_window_corner_points(corner_pts, img)
		print ("imgPoints = ",imgPoints)
		rotVec,transVec = self.pnp(imgPoints,img)
			
		# if cv2.waitKey(1)& 0xff==ord('q'):
		# 	cv2.destroyAllWindows()
			# count += 1

	def detection_hough_lines(self,original_img, mask):
		# img = cv2.imread(self.original_image)
		# mask = cv2.imread(self.image_path,0)
		n_rows = int(original_img.shape[0]/4)
		n_cols = int(original_img.shape[1]/4)	
		img = cv2.resize(original_img, (n_cols, n_rows))
		mask = cv2.inRange(mask, 150, 255)
		
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
		# closing to fill unwanted small gaps
		closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
		gray = cv2.medianBlur(closing,3)
		
		masked_img = cv2.bitwise_and(img, img, mask = closing)
		# print mask.shape
		# print img.shape

		gray = cv2.cvtColor(masked_img,cv2.COLOR_BGR2GRAY)
		# edges = gray
		
		gray = cv2.GaussianBlur(gray, (3,3), cv2.BORDER_DEFAULT)
		# edges = cv2.Canny(gray, 50, 150, apertureSize = 3)
		minLength = 200
		maxLineGap = 100
		new_y = int(original_img.shape[0])
		new_x = int(original_img.shape[1])	
		edges = cv2.resize(gray, (new_x, new_y))
		lines = cv2.HoughLinesP(edges,10, np.pi/180, 150, minLength, maxLineGap)
		# lines = cv2.HoughLines(gray, 1, np.pi/180, 10, None, 0, 0 )
		# print lines.shape

		# input('as')
		houghlines = np.zeros_like(original_img)
		try:
			for x1, y1, x2, y2 in lines[:,0,:]:
				cv2.line(houghlines, (x1, y1),(x2,y2), (0,255,0), 2)
		except:
			print('lines not found')
			pass

		# print ("imgPoints = ",imgPoints)

		kernel_lines = cv2.getStructuringElement(cv2.MORPH_RECT,(31,31))
		# closing to fill unwanted small gaps
		houghlines = cv2.morphologyEx(houghlines, cv2.MORPH_CLOSE, kernel_lines)
		houghlines = cv2.medianBlur(houghlines,3)
		houghlines_gray = cv2.cvtColor(houghlines, cv2.COLOR_RGB2GRAY)

		##############################################################
		#find contours
		houghlines_gray, contours, hierarchy = cv2.findContours(houghlines_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		epsilon = 0.1*cv2.arcLength(contours[0], True)
		approx = cv2.approxPolyDP(contours[0], epsilon, True)
		if ((len(approx) >= 4)):
			points = np.squeeze(np.array(approx))
			if cv2.isContourConvex(points):
				corners = self.get_outer_window_corner_points(points, original_img)
				cv2.drawContours(original_img, [approx], 0, (0,0,0), 3)
				self.center = np.vstack([self.center, np.mean([self.top_left,self.top_right,self.bottom_right, self.bottom_left],axis=0)])
				self.top_left_corner = np.vstack([self.top_left_corner, corners[0]])
				self.top_right_corner = np.vstack([self.top_right_corner, corners[1]])
				self.bottom_left_corner = np.vstack([self.bottom_left_corner, corners[3]])
				self.bottom_right_corner = np.vstack([self.bottom_right_corner, corners[2]])

				if self.top_left_corner.shape[0] > 5:
					self.top_left_corner = np.delete(self.top_left_corner,0,0)
					self.top_right_corner = np.delete(self.top_right_corner,0,0)
					self.bottom_left_corner = np.delete(self.bottom_left_corner,0,0)
					self.bottom_right_corner = np.delete(self.bottom_right_corner,0,0)
					
				if self.center.shape[0] > 10:
					self.center = np.delete(self.center,0,0)

				# print(self.center.shape)
				self.centerpoint = np.mean(self.center,axis=0)
				self.top_left = np.mean(self.top_left_corner, axis=0)
				self.top_right = np.mean(self.top_right_corner, axis=0)
				self.bottom_left = np.mean(self.bottom_left_corner, axis=0)
				self.bottom_right = np.mean(self.bottom_right_corner, axis=0)

				self.goodCorners = np.array([self.top_left, self.top_right, self.bottom_right, self.bottom_left])
				rotVec,transVec = self.pnp(self.goodCorners,original_img)

		else:
			pass
		# corner_pts = self.get_all_corners(houghlines, original_img)
		# imgPoints = self.get_outer_window_corner_points(corner_pts, original_img)
		# centerpoint = np.mean(imgPoints, axis=0)
		# print(corner_pts.shape)
		# input('as')
		cv2.circle(original_img, tuple((int(self.centerpoint[0]), int(self.centerpoint[1]))), 5, (0,0,0), -1)

		#corners
		cv2.circle(original_img, tuple((int(self.top_left[0]), int(self.top_left[1]))), 5, (0,0,0), -1)
		cv2.circle(original_img, tuple((int(self.top_right[0]), int(self.top_right[1]))), 5, (0,0,0), -1)
		cv2.circle(original_img, tuple((int(self.bottom_right[0]), int(self.bottom_right[1]))), 5, (0,0,0), -1)
		cv2.circle(original_img, tuple((int(self.bottom_left[0]), int(self.bottom_left[1]))), 5, (0,0,0), -1)

		# for i in range(imgPoints.shape[0]):
		# 	pt = imgPoints[i]
		# 	cv2.circle(original_img,tuple(pt),3,(0,0,255),-1)
		# except:
		# 	pass

		cv2.imshow('frame',original_img)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			cv2.destroyAllWindows()
		# masked_img = cv2.resize(img, (int(img.shape[1]/2), (int(img.shape[0]/2))))
		cv_image = self.bridge.cv2_to_imgmsg(original_img, "bgr8")
		self.image_pub.publish(cv_image)

	def Detection_using_threshold_image(self, mask):
		# img = cv2.imread(self.original_image)
		# mask = cv2.imread(self.image_path,0)
		mask = cv2.inRange(mask, 200, 255)
		masked_img = cv2.bitwise_and(img, img, mask = mask)
		print(masked_img.shape)
		corner_pts = self.get_all_corners(masked_img, img)
		imgPoints = self.get_outer_window_corner_points(corner_pts, img)
		print ("imgPoints = ",imgPoints)
		rotVec,transVec = self.pnp(imgPoints,img)
				
		cv2.waitKey(0)




def main():
	count = 0
	ob = video_stream()

	rospy.init_node('image_reader', anonymous=True)
	while(not rospy.is_shutdown()):
		rospy.spin()
		count += 1;

if __name__ == '__main__':
	main()
