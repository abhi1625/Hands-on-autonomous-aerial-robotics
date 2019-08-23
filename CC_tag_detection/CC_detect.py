#!/usr/bin/env python
try:
		sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages/')
except:
		pass

import cv2
import numpy as np
import math
import copy

class BullsEyeDetection:
		def __init__(self):
				self.data_path = '../../drone_course_data/CC_tag_detection/CCTAG.jpg'


		# def detect_ellipse(self,img):
		# 		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# 		gray = cv2.medianBlur(gray,5)
		# 		edges = cv2.Canny(gray,50,150,apertureSize = 3)
		# 		cv2.imshow('edges', edges)
		# 		cv2.waitKey(0)
		# 		cv2.destroyAllWindows()
		# 		i,contours,h = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
		# 		# print("h = ", h)
		# 		max_w = 0
		# 		max_h = 0
		# 		iter_ = -1
		# 		count = 0
		# 		print("contours",np.shape(contours))
		# 		for i,c1 in enumerate(contours):
		# 				(x,y,we,he) = cv2.boundingRect(c1)

		# 				if len(c1)>4 and (we > 150 and he > 150):
		# 						if he>max_h or we>max_w:
		# 								max_h = he
		# 								max_w = we
		# 								iter_=i
		# 						print i
		# 						(cx,cy),(Ma,ma),th = cv2.fitEllipse(c1)
		# 						print("ellipse = ",(cx,cy),(Ma,ma),th)
		# 						# if iter_ == 855:
		# 						#     print c1.shape
		# 						cv2.drawContours(img, c1, -1, (0,255,0), 2)
		# 								# continue
		# 						# print ("ellipse",ellipse)
		# 						count+=1
		# 		# cv2.drawContours(img, c1, -1, (0,255,0), 2)
		# 		print ("count = ",count)
		# 		print iter_
		# 		cv2.imshow('img', img)
		# 		cv2.waitKey(0)
		# 		cv2.destroyAllWindows()
							 # if w > 80 and h > 80:
							 #       approx = cv2.approxPolyDP(c1, 0.01*cv2.arcLength(c1, True), True)
							 #       if(len(approx) > 10):
							 #              contourFoundRed = True
							 #              M = cv2.moments(c1)
							 #              cX = int(M["m10"] / M["m00"])
							 #              cY = int(M["m01"] / M["m00"])
							 #              #cv2.ellipse(frame,ellipse,(0,255,255),2)
							 #              (xf,yf,wf,hf) = cv2.boundingRect(c1)
							 #              #cv2.putText(frame,'Dropoff Detected', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0),2, lineType=cv2.LINE_AA)
							 #              #cv2.drawContours(frame, c1, -1, (0,255,0), 2)
							 #              redContour.append(ellipse)
							 #              redCenter.append((cX,cY))
							 #              break

				
		def detect_ellipse_hough(self,img):
				# gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
				gray = cv2.medianBlur(img,5)
				cimg = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
				# edges = cv2.Canny(gray,50,150,apertureSize = 3)
				# cv2.imshow('edges', edges)
				# cv2.waitKey(0)
				# cv2.destroyAllWindows()
				circles = cv2.HoughCircles(cimg,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
				circles = np.uint16(np.around(circles))
				for i in circles[0,:]:
					# draw the outer circle
					cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
					# draw the center of the circle
					cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
				# print("h = ", h)
				max_w = 0
				max_h = 0
				iter_ = -1
				count = 0
				
				cv2.imshow('cimg', cimg)
				cv2.waitKey(0)
				cv2.destroyAllWindows()

		def run_pipeline(self):
				img = cv2.imread(self.data_path)
				h,w,_ = img.shape
				dim = (w/4,h/4)
				img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
				print("img size = ",img.shape)
				self.detect_ellipse_hough(img)


def main():
		ob = BullsEyeDetection()
		ob.run_pipeline()


if __name__ == '__main__':
		main()