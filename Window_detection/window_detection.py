#!/usr/bin/env python
import rospy
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
import GMM.test_data

class video_stream:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_raw", Image, self.img_callback)
		self.window_detection = Window_detection()
		weights_path = './training_params/window_weights.npy'
		params_path = './training_params/gaussian_params.npy'
		self.n, self.K, self.weights, self.params = loadparamsGMM(weights_path, params_path)

	def img_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		if cv_image is not None:
			processed_img = preprocess_img(cv_image)

			#run GMM inference to generate mask
			mask = test_combined(processed_img,self.K,self.n,self.weights, self.params,(0,255,0))
			self.window_detection.detection_hough_lines(mask)
	
class Window_detection:
	def __init__(self):
		# self.data_path = '/home/prgumd/Desktop/pink_window.mp4'
		self.data_path = '../../drone_course_data/window_detection/pink_window.mp4'
		self.image_path = './GMM/test_image.jpg'
		self.original_image = '../drone_course_data/Window_detection/GMM/data/GMM_1/frame0200.jpg'

	def rotate_img_90_deg(self,img):
		h,w = img.shape[:2]
		center = (w/2,h/2)
		M = cv2.getRotationMatrix2D(center, -90, 0.5)
		M[0,-1] = M[0,-1] - ((w-h/2)/2)
		M[-1,-1] = M[-1,-1] - ((h-w/2)/2)
		warped_img = cv2.warpAffine(copy.deepcopy(img), M, (h/2,w/2))
		return warped_img

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
		for pts in corners:
			pt = pts[0]
			cv2.circle(closing,tuple(pt),3,255,-1)

		cv2.imshow('test', closing)
		cv2.waitKey(0)
		cv2.destroyAllWindows()


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
		corner_pts = np.delete(corner_pts,bottom_most_pt,0)


		distances = (corner_pts[:,0]-w)**2 + (corner_pts[:,1]-h)**2
		right_bottom_corner = corner_pts[np.argmin(distances),:]

		distances = corner_pts[:,0]**2 + (corner_pts[:,1]-h)**2
		left_bottom_corner = corner_pts[np.argmin(distances),:]

		imgPoints = np.array([[left_top_corner],
							  [right_top_corner],
							  [right_bottom_corner],
							  [left_bottom_corner]],dtype=np.float64)
		imgPoints = np.squeeze(imgPoints)
		return imgPoints

	def pnp(self, imgPoints,img):
		h,w = img.shape[:2]
		# World coordinates using window measurement in world
		objPoints = np.array([[0,0,0],
							 [61,0,0],
							 [61,91,0],
							 [0,91,0]], dtype=np.float64)

		# Camera K matrix(intrinsic params)
		camMatrix = np.array([[400, 0 , w/2],[0, 400, h/2],[0,0,1]],dtype=np.float32)

		#distortion coefficients 
		distCoeffs = np.array([0,0,0,0,0],dtype=np.float64)

		_, rotVec, transVec = cv2.solvePnP(objPoints,imgPoints, camMatrix, distCoeffs)

		print(rotVec, transVec)

		# Verification by reporjecting points using rotation and 
		# translation computed above
		reprojPoints,_ = cv2.projectPoints(objPoints, rotVec, transVec, camMatrix, distCoeffs)
		reprojPoints = np.squeeze(reprojPoints)
		print(reprojPoints.shape)

		for i in range(4):
		        pt = reprojPoints[i]
		        imgpt = imgPoints[i]
		        cv2.circle(img, (int(pt[0]), int(pt[1])),5,[255,0,0],-1)
		        cv2.circle(img, (int(imgpt[0]),int(imgpt[1])),5,[0,255,0],-1)
		cv2.imshow('image',img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

		return rotVec,transVec


	def Detection_using_threshold(self):
		vidcap = cv2.VideoCapture(self.data_path)
		count = 0
		while count <250:
			success,img = vidcap.read()
		# while count<1:
			if((count<250) and(count%40 ==0)):
				# print("image dim = ",np.shape(img))
				print('Read frame # ', count+1)
				
				img = self.rotate_img_90_deg(img)
				img_pink = np.logical_and(np.logical_and(img[:,:,2]>200,img[:,:,0]>130,img[:,:,0]<200),img[:,:,1]<150)
				
				img_pink = np.dstack((img_pink,img_pink,img_pink))
				img_pink = img_pink*img

				corner_pts = self.get_all_corners(img_pink,img)
				imgPoints = self.get_outer_window_corner_points(corner_pts, img)
				print ("imgPoints = ",imgPoints)
				rotVec,transVec = self.pnp(imgPoints,img)
				
			if cv2.waitKey(1)& 0xff==ord('q'):
				cv2.destroyAllWindows()
			count += 1

	def detection_hough_lines(self, mask):
		# img = cv2.imread(self.original_image)
		# mask = cv2.imread(self.image_path,0)
		mask = cv2.inRange(mask, 200, 255)
		masked_img = cv2.bitwise_and(img, img, mask = mask)

		gray = cv2.cvtColor(masked_img,cv2.COLOR_BGR2GRAY)
		gray = cv2.medianBlur(gray,5)
		edges = gray
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(31,31))
		# closing to fill unwanted small gaps
		closing = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
		
		closing = cv2.GaussianBlur(closing, (7,7), cv2.BORDER_DEFAULT)
		edges = cv2.Canny(closing, 50, 150, apertureSize = 3)
		minLength = 100
		maxLineGap = 40
		lines = cv2.HoughLinesP(closing, 1, np.pi/180, 100, minLength, maxLineGap)
		# print lines.shape
		# input('as')
		for x1, y1, x2, y2 in lines[:,0,:]:
			cv2.line(img, (x1, y1),(x2,y2), (0,255,0), 2)
		# print ("imgPoints = ",imgPoints)
		cv2.imshow("image", img)				
		cv2.waitKey(0)

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

	rospy.init_node('image_reader', anonymus=True)
	while(count != 1000 || not rospy.is_shutdown()):
		rospy.spin()
		count += 1;

if __name__ == '__main__':
	main()
