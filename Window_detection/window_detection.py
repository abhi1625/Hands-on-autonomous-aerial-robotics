#!/usr/bin/env python

import cv2
import numpy as np
import math
import copy

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

	def get_hough_lines(self,img,thresh = 100):
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray,(9,9),cv2.BORDER_DEFAULT)
		edges = cv2.Canny(gray,50,150,apertureSize = 3)
		cv2.imshow('image',edges)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		h,w = gray.shape
		# lines = cv2.HoughLines(edges,np.sqrt(w**2+h**2),np.pi/180,thresh)
		minLineLen = np.maximum(w,h)*0.3
		rho = np.sqrt(w**2+h**2)
		# print("lines= ",lines)
		try:
			lines = cv2.HoughLinesP(edges, rho , np.pi/180, thresh, minLineLength=minLineLen, maxLineGap=250)
			for line in lines:
			    x1, y1, x2, y2 = line[0]
			    cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)

		except TypeError as TE:
			thresh = int(0.8*thresh)
			lines = cv2.HoughLinesP(edges, rho, np.pi/180, thresh, minLineLength=minLineLen, maxLineGap=250)
			for line in lines:
			    x1, y1, x2, y2 = line[0]
			    cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)			
		else:
			pass
		finally:
			pass
		return img



	def Detection_using_threshold(self):
		vidcap = cv2.VideoCapture(self.data_path)
		success,img = vidcap.read()
		count = 0
		while success:
		# while count<1:
			if((count%40 ==0) and count <50):
				# print("image dim = ",np.shape(img))
				print('Read frame # ', count+1)
				
				# cv2.imshow('image',img)
				# cv2.waitKey(0)
				# cv2.destroyAllWindows()
				img = self.rotate_img_90_deg(img)
				# img_pink = np.logical_and(img[:,:,2]>135,img[:,:,0]>130)
				img_pink = np.logical_and(img[:,:,2]>185,img[:,:,0]>130)
				# img_pink = np.logical_and(img[:,:,2]>185,img[:,:,0]>120, img[:,:,1]<100)
				img_pink = np.dstack((img_pink,img_pink,img_pink))
				img_pink = img_pink*img

				img_pink = self.get_hough_lines(img_pink,thresh = 4000)

				cv2.imshow('image',img_pink)
				cv2.waitKey(0)
				cv2.destroyAllWindows()
			success,img = vidcap.read()
			count += 1


def main():
	ob = Window_detection()
	ob.Detection_using_threshold()

if __name__ == '__main__':
	main()
