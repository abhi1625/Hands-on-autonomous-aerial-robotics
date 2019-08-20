#!/usr/bin/env python
try:
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages/')
except:
	pass

import cv2
import numpy as np
import math
import copy
from sklearn import linear_model, datasets


class Window_detection:
	def __init__(self):
		self.data_path = '/home/pratique/drone_course_data/window_detection/pink_window.mp4'

	def rotate_img_90_deg(self,img):
		h,w = img.shape[:2]
		center = (w/2,h/2)
		M = cv2.getRotationMatrix2D(center, -90, 0.5)
		M[0,-1] = M[0,-1] - ((w-h/2)/2)
		M[-1,-1] = M[-1,-1] - ((h-w/2)/2)
		warped_img = cv2.warpAffine(copy.deepcopy(img), M, (h/2,w/2))
		return warped_img

	def get_hough_lines(self,img,img_orig,thresh = 1):
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# gray = cv2.GaussianBlur(gray,(7,7),cv2.BORDER_DEFAULT)
		gray = cv2.medianBlur(gray,5)

		# edges = cv2.Canny(gray,50,150,apertureSize = 3)
		edges = gray
		# print("canny type", np.shape(edges))
		cv2.imshow('image',edges)
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(31,31))
		closing = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
		B_mask = np.uint8((closing[:]>100)*255)




		# dilation = cv2.dilate(B_mask,kernel,iterations = 1)
		# cv2.imshow('dilation', dilation)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		# closing = np.uint8((dilation[:]>200)*cv2.cvtColor(img_orig,cv2.COLOR_BGR2GRAY))

		# cv2.imshow('closing', closing)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		
		closing_3d = np.dstack((closing,closing,closing))

		# img,h,contours = cv2.findContours(closing,1,2)
		# print(contours)
		# input('a')
		# for cnt in contours:
		#     approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
		#     print len(approx)
		#     if len(approx)==5:
		#         print "pentagon"
		#         cv2.drawContours(closing_3d,[cnt],0,(0,255,0),-1)
		#     elif len(approx)==3:
		#         print "triangle"
		#         cv2.drawContours(closing_3d,[cnt],0,(0,255,0),-1)
		#     elif len(approx)==4:
		#         print "square"
		#         cv2.drawContours(closing_3d,[cnt],0,(0,0,255),-1)
		#     elif len(approx) == 9:
		#         print "half-circle"
		#         cv2.drawContours(closing_3d,[cnt],0,(255,255,0),-1)
		#     elif len(approx) > 15:
		#         print "circle"
		#         cv2.drawContours(closing_3d,[cnt],0,(0,255,255),-1)
		# dst = cv2.cornerHarris(np.float32(closing),2,3,0.04)
		# print("sdt",dst)
		# dst = cv2.dilate(dst,None)
		# closing_3d[dst>0.1*dst.max()]=[0,0,255]
		closing = cv2.GaussianBlur(closing, (7,7), cv2.BORDER_DEFAULT)
		corners = cv2.goodFeaturesToTrack(closing, 111, 0.1, 30)
		# print(corners)
		for pts in corners:
			pt = pts[0]
			cv2.circle(closing,tuple(pt),3,255,-1)

		cv2.imshow('test', closing)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

		# h,w = gray.shape
		# # lines = cv2.HoughLines(edges,np.sqrt(w**2+h**2),np.pi/180,thresh)
		# # print("lines= ",lines)
		# corner_img = np.zeros_like(closing)
		# # for pts in corners:
		# # 	pt = pts[0]
		# # 	cv2.circle(corner_img,tuple(pt),3,255,-1)

		# # cv2.imshow('test', corner_img)
		# # cv2.waitKey(0)
		# # cv2.destroyAllWindows()

		# minLineLen = 200
		# rho = 1# np.sqrt(w**2+h**2)
		# maxLineGap = 150
		# theta = np.pi/180
		# thresh = 100
		# try:
		# 	lines = cv2.HoughLinesP(closing, rho , theta, thresh, minLineLength= minLineLen, maxLineGap=maxLineGap)
		# 	lines = np.squeeze(lines)
		# 	for line in lines:
		# 	    x1, y1, x2, y2 = line
		# 	    cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
		# try:
		# 	lines = cv2.HoughLines(cv2.cvtColor(img_orig,cv2.COLOR_BGR2GRAY),1, theta,1000)
		# 	# print(np.shape(lines))
		# 	# input('qw')
		# 	lines = np.squeeze(lines)
		# 	# print(np.shape(lines))
		# 	# input('qs')
		# 	for rho,theta in lines:
		# 		# print(lines[0, :])
		# 		# print(rho)
		# 		# print(theta)
		# 		# input('qq')
		# 		a = np.cos(theta)
		# 		b = np.sin(theta)
		# 		x0 = a*rho
		# 		y0 = b*rho
		# 		x1 = int(x0 + 1000*(-b))
		# 		y1 = int(y0 + 1000*(a))
		# 		x2 = int(x0 - 1000*(-b))
		# 		y2 = int(y0 - 1000*(a))

		# 		cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
		# except TypeError as TE:
		# 	pass
			# thresh = int(0.8*thresh)
			# lines = cv2.HoughLinesP(closing, rho, thetha, thresh, minLineLength=minLineLen, maxLineGap=maxLineGap)

			# for line in lines:
			#     x1, y1, x2, y2 = line[0]
			#     cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)


		return np.squeeze(corners)



	def Detection_using_threshold(self):
		vidcap = cv2.VideoCapture(self.data_path)
		count = 0
		while count <40:
			success,img = vidcap.read()
		# while count<1:
			if((count<40) and(count%40 ==0)):
				# print("image dim = ",np.shape(img))
				print('Read frame # ', count+1)
				
				img = self.rotate_img_90_deg(img)
				# img_pink = np.logical_and(img[:,:,2]>135,img[:,:,0]>130)
				img_pink = np.logical_and(np.logical_and(img[:,:,2]>200,img[:,:,0]>130,img[:,:,0]<200),img[:,:,1]<150)
				# img_pink = np.logical_and(img[:,:,2]>185,img[:,:,0]>120, img[:,:,1]<100)
				img_pink = np.dstack((img_pink,img_pink,img_pink))
				img_pink = img_pink*img
				cv2.imshow('image',img_pink)
				cv2.waitKey(0)
				cv2.destroyAllWindows()

				corner_pts = self.get_hough_lines(img_pink,img)
				self.pnp(corner_pts)
				print(corner_pts.shape)
			if cv2.waitKey(1)& 0xff==ord('q'):
				cv2.destroyAllWindows()
			count += 1



def main():
	ob = Window_detection()
	ob.Detection_using_threshold()

if __name__ == '__main__':
	main()
