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

class BullsEyeDetection:
	def __init__(self):
                self.data_path = '/home/pratique/drone_course_data/CC_tag_detection'
                self.thresh = 0.8

	def detect_ellipse_fitellipse(self,img):
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		max_val = np.amax(gray)
		rand_thresh = gray.copy()
		rand_thresh[rand_thresh[:]<(max_val*self.thresh)] = 0 
		edges = cv2.Canny(rand_thresh,50,150,apertureSize = 3)
		# kernel = np.ones((2,2),np.uint8)
		# edges = cv2.dilate(edges,kernel,iterations = 1)
		cv2.imshow('edges', edges)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		
		# minLineLength = 40
		# maxLineGap = 100
		# try:
		lines = cv2.HoughLines(edges,1,np.pi/180,30)
		same_thresh = 0.25
		orthogonal_thresh =0.5

		if(lines is not None):
			# rho_arr= np.zeros((1,),dtype=float)
			# print("lines shape",lines.shape)
			# lines = np.squeeze(lines)
			# rho_arr = lines[:,1]
			# clusters = []
			# while rho_arr is not None:
			# 	k1 = []
			# 	comp = rho_arr[0]
			# 	for each in rho_arr:
					


			print("lines shape",lines.shape)
			for rho,theta in lines:
				print("angle = ", theta)
				a = np.cos(theta)
				b = np.sin(theta)
				x0 = a*rho
				y0 = b*rho
				x1 = int(x0 + 1000*(-b))
				y1 = int(y0 + 1000*(a))
				x2 = int(x0 - 1000*(-b))
				y2 = int(y0 - 1000*(a))

				cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
				cv2.imshow('edges', img)
				cv2.waitKey(0)
				cv2.destroyAllWindows()
		
		# i,contours,h = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
		# # print("h = ", h)
		# max_w = 0
		# max_h = 0
		# iter_ = -1
		# count = 0
		# print("contours",np.shape(contours),np.shape(contours[1:]))
		# for i,c1 in enumerate(contours[1:]):
		# 		print i
		# 		# print("c1 = ", c1)
		# 		(x,y,we,he) = cv2.boundingRect(c1)
		# 		print("bb params = ",x,y,we,he)

		# 		if len(c1)>4 and (we > 10 and he > 10):
		# 				if he>max_h or we>max_w:
		# 						max_h = he
		# 						max_w = we
		# 						iter_=i
		# 				(cx,cy),(Ma,ma),th = cv2.fitEllipse(c1)
		# 				print("ellipse = ",(cx,cy),(Ma,ma),th)
		# 				# if iter_ == 855:
		# 				#     print c1.shape
		# 				cv2.drawContours(img, c1, -1, (0,255,0), 2)
		# 						# continue
		# 				# print ("ellipse",ellipse)
		# 				cv2.imshow('drawContours', img)
		# 				cv2.waitKey(0)
		# 				cv2.destroyAllWindows()
		# 				count+=1
		# # cv2.drawContours(img, c1, -1, (0,255,0), 2)


	def detect_ellipse_hough(self,img):
		kernel = np.ones((3,3),np.uint8)
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# gray = cv2.medianBlur(gray,5)
		# print("gray shape = ",gray.shape)
		# laplacian = cv2.Laplacian(gray,cv2.CV_64F,ksize = 5)
		# laplacian = cv2.bitwise_not(laplacian)
		max_val = np.amax(gray)
		rand_thresh = gray.copy()
		rand_thresh[rand_thresh[:]<(max_val*self.thresh)] = 0 
		                #cimg = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
		# eroded_img = cv2.erode(rand_thresh,kernel,iterations = 1)
		# edges = cv2.Canny(gray,120,200,apertureSize = 3)
		eroded_img = rand_thresh
		cv2.imshow('edges', rand_thresh)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		cv2.imshow('eroded', eroded_img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		img_three = eroded_img.copy()
		img_three = np.dstack((img_three,eroded_img,eroded_img))
		print("3d img shape = ",img_three.shape)
		circles = cv2.HoughCircles(eroded_img,cv2.HOUGH_GRADIENT,1,10,param1=150,param2=50,minRadius=0,maxRadius=0)
		print("circles = ",circles)
		                #input("aaa")
		if(circles is not None):
				
			circles = np.uint16(np.around(circles))
			circles = np.squeeze(circles)
			print("circle shape = ",circles.shape)
			if (circles.shape[0] == 3):
			        # draw the outer circle
			    cv2.circle(img_three,(circles[0],circles[1]),circles[2],(0,255,0),2)
			    # draw the center of the circle
			    cv2.circle(img_three,(circles[0],circles[1]),2,(0,0,255),3)
			else:

				for i in circles:
				        # draw the outer circle
				    cv2.circle(img_three,(i[0],i[1]),i[2],(0,255,0),2)
				    # draw the center of the circle
				    cv2.circle(img_three,(i[0],i[1]),2,(0,0,255),3)
			cv2.imshow('circles', img_three)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
			# input("ad")
			max_w = 0
			max_h = 0
			iter_ = -1
			count = 0
		else:
			print("no circles detected")
		
		# cv2.imshow('cimg', cimg)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()

	def run_pipeline(self):
		dirname = sorted(os.listdir(self.data_path))
		for filename in dirname:
		    #print("filename",filename)
		    #print(os.path.join(self.data_path,filename))
		    img = cv2.imread(os.path.join(self.data_path,filename))
		    #cv2.imshow("read img",img)
		#cv2.waitKey(0)
		#cv2.destroyAllWindows()
		    h,w,_ = img.shape
		    #dim = (w/4,h/4)
		    #img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
		    print("img size = ",img.shape)
		    # self.detect_ellipse_hough(img)
		    self.detect_ellipse_fitellipse(img)


def main():
		ob = BullsEyeDetection()
		ob.run_pipeline()


if __name__ == '__main__':
		main()
