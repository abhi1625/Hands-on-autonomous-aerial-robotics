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

class BullsEyeDetection:
	def __init__(self):
		self.data_path = '/home/pratique/drone_course_data/CC_tag_detection'
		self.thresh = 0.9
		self.tag_rad = 0.2075    #m
		self.image_sub = rospy.Subscriber("/duo3d/left/image_rect", Image, self.img_callback)
		self.pose_pub = rospy.Publisher("/cctag_sp", Pose, queue_size = 1)
		self.image = None
		self.pose_obj = Pose()
		self.pose_obj.position.x = 0.0			#in m
		self.pose_obj.position.y = 0.0 	#in m
		self.pose_obj.position.z = 0.0  	#in m
		self.pose_obj.orientation.x = 0
		self.pose_obj.orientation.y = 0
		self.pose_obj.orientation.z = 0
		self.bridge = CvBridge()
		self.detect_flag = True
		self.detect_sub = rospy.Subscriber("/cctag_detect", Bool, self.detect_flag_cb)
		self.centers = np.zeros((1,2))
		self.filter_len = 5

	def detect_flag_cb(self,data):
		self.detect_flag = data.data
		print("detect flag",self.detect_flag)
	def img_callback(self, data):
		try:
			self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# print('got frame')
		except CvBridgeError as e:
			print(e)
	
	def LinearPnP(X,x,K, kc):
		homo_x = np.insert(x, 2, 1,axis =1).T
		norm_x = np.matmul(np.linalg.inv(K),homo_x)
		norm_x = norm_x/norm_x[2]
		norm_x = norm_x.T
		#print(norm_x)

		A = np.zeros([1,12])

		for pt in range(x.shape[0]):
			mat = np.array([[-X[pt][0], -X[pt][1], -X[pt][2], -1,0,0,0,0, norm_x[pt][0]*X[pt][0],norm_x[pt][0]*X[pt][1], norm_x[pt][0]*X[pt][2],
							norm_x[pt][0]*1 ],
							[0,0,0,0,-X[pt][0],-X[pt][1],-X[pt][2],-1, norm_x[pt][1]*X[pt][0],norm_x[pt][1]*X[pt][1], norm_x[pt][1]*X[pt][2],norm_x[pt][1]*1]])
							# [-norm_x[pt][1]*X[pt][0], -norm_x[pt][1]*X[pt][1], -norm_x[pt][1]*X[pt][2],
							# -norm_x[pt][1]*1, norm_x[pt][0]*X[pt][0], norm_x[pt][0]*X[pt][1], norm_x[pt][0]*X[pt][2], norm_x[pt][0]*1,0,0,0,0  ]])

			#print("mat shape",mat.shape)
			A = np.concatenate((A,mat),axis=0)
		A = A[1:,:]
		#print("A matrix",A)
		U,S,Vh = np.linalg.svd(A)
		P = Vh[-1]
		#print("svd V",Vh)
		P = np.reshape(P,(3,4))
		# P = P.T
		# print(P)
		Rnew = P[:,:3]
		u,s,vh = np.linalg.svd(Rnew)
		Rnew = np.matmul(u,vh)

		Cnew = np.matmul(-np.linalg.inv(Rnew),P[:,3])
		if np.linalg.det(Rnew)<0:
			# print(np.linalg.det(Rnew))
			Rnew = -Rnew
			Cnew = -Cnew
		#     print('R',Rnew.shape)
		return Cnew, Rnew


	def get_line_clusters(self,np_lines,rho_thresh,theta_thresh):
		line_clusters = []
		# print("np lines = ",np_lines)
		while(np_lines.shape[0] >2 ):
			comp_theta = np_lines[0,1]
			comp_rho = np_lines[0,0]

			group = np.zeros((1,2))
			count = 0
			for i,[rho, theta] in enumerate(np_lines):
				if abs(comp_theta-theta)<theta_thresh and abs(comp_rho - rho)<rho_thresh:
					group = np.vstack((group,np.array(np_lines[i-count,:])))
					np_lines = np.delete(np_lines,i-count,0)
					count +=1
			group = group[1:,:]
			group_mean = np.mean(group,axis = 0)

			line_clusters.append(group_mean)
		return np.array(line_clusters)

	def augment_lines(self,line_clusters):
		
		rho_arr = line_clusters[:,0]
		# print("rho_arr = ",rho_arr)
		try:
			i_max = int(np.where(rho_arr == np.amax(rho_arr))[0])
		except:
			i_max = int(np.where(rho_arr == np.amax(rho_arr))[0][0])
		try:
			i_min = int(np.where(rho_arr == np.amin(rho_arr))[0])
		except:
			i_min = int(np.where(rho_arr == np.amin(rho_arr))[0][0])


		max_line = line_clusters[i_max]
		min_line = line_clusters[i_min]
		line_clusters = np.delete(line_clusters,i_max,0)
		line_clusters = np.delete(line_clusters,i_min-1,0)
		rho_arr = np.delete(rho_arr,i_max,0)
		rho_arr = np.delete(rho_arr,i_min-1,0)
		print("rho_arr = ",rho_arr)
		try:
			i_max_2 = int(np.where(rho_arr == np.amax(rho_arr))[0])
		except:
			i_max_2 = int(np.where(rho_arr == np.amax(rho_arr))[0][0])

		try:
			i_min_2 = int(np.where(rho_arr == np.amin(rho_arr))[0])
		except:
			i_min_2 = int(np.where(rho_arr == np.amin(rho_arr))[0][0])

		sec_max_line = line_clusters[i_max_2]
		sec_min_line = line_clusters[i_min_2]
		sec_max_line[0] = sec_max_line[0]+3
		max_line[0] = max_line[0]-3
		sec_min_line[0] = sec_min_line[0]-3
		min_line[0] = min_line[0]+3
		line_clusters = [max_line,sec_max_line,sec_min_line,min_line]
		
		return line_clusters



	def get_hough_lines(self,edges):
		kernel = np.ones((11,11),np.uint8)
		edges = cv2.dilate(edges,kernel,iterations = 1)
		# cv2.imshow('dilated edges in hough', edges)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		# kernel = np.ones((2,2),np.uint8)
		# dilated_edges = cv2.dilate(edges,kernel,iterations = 1)

		minLength = 200
		maxLineGap = 50
		# lines = cv2.HoughLines(edges,1,np.pi/120,40)
		lines = cv2.HoughLinesP(edges,1,np.pi/120,10, minLength, maxLineGap)


		if(lines is not None):
			# lines = np.squeeze(lines)
			print("houghlinesP shape =", lines.shape)
			edges3CH = np.dstack((edges,edges,edges))
			new_lines = []
			for x1, y1, x2, y2 in lines[:,0,:]:
				m = (y1-y2)/(x1-x2)
				th = math.atan2(-1,m)
				rh = (-m*x1+y1)*math.sin(th)
				cv2.line(edges3CH,(x1,y1),(x2,y2),(0,0,255),2)
				# rect_line = rect_line[1:,:]
				# print ("rect_line",rect_line)
				new_lines.append([rh,th])
			# cv2.imshow('edges3CH', edges3CH)
			# cv2.waitKey(1)
			# print("new_lines = ",new_lines)
			np_lines=np.array(new_lines)
			status = False
			line_clusters = self.get_line_clusters(np_lines,25,0.4)
			print("lines clusters = ", line_clusters)
			if(line_clusters.shape[0]>=4):
				mod_line_cluster = self.augment_lines(line_clusters)

				# print ("mod_line_cluster",mod_line_cluster)
				rect_line = np.zeros((1,2))
				
				# print line_clusters
				# for rho,theta in mod_line_cluster:
				for rho,theta in new_lines:

					# rho = rho+1
					a = np.cos(np.float(theta))
					b = np.sin(np.float(theta))
					##### convert line from polar coordinates to cartesian coordinattes
					slope = -a/b
					intercept = rho/b
					# print("intercept, slope = ",intercept,slope)
					rect_line = np.vstack((rect_line,np.array([intercept,slope])))
					x0 = a*rho
					y0 = b*rho
					x1 = int(x0 + 1000*(-b))
					y1 = int(y0 + 1000*(a))
					x2 = int(x0 - 1000*(-b))
					y2 = int(y0 - 1000*(a))
					# cv2.line(edges3CH,(x1,y1),(x2,y2),(0,0,255),2)
				rect_line = rect_line[1:,:]
				# print ("rect_line",rect_line)
				# cv2.imshow('edges3CH', edges3CH)
				# cv2.waitKey(1)
				# cv2.destroyAllWindows()
				status = True
				return status,rect_line
			else:
				# print('no hough lines')
				status = False
				return status,lines

		else:
			status = False
			return status,lines

	def refine_contours(self,img,rect_line):
		h,w = img.shape
		mesh = np.mgrid[0:h,0:w]
		# print ("mesh shape = ", mesh.shape)
		rows = mesh[0,:,:]
		cols = mesh[1,:,:]
		# print("row and col shape", rows.shape, cols.shape)
		for i,[intercept,slope] in enumerate(rect_line):
			if(i == 1):
				img[rows-slope*cols-intercept>0] = 0
			if(i == 3):
				img[rows-slope*cols-intercept<0] = 0
			if(i == 2):
				img[rows-slope*cols-intercept>0] = 0
			if(i == 0):
				img[rows-slope*cols-intercept<0] = 0
			
		# intercept0,slope0 = rect_line[0]
		# intercept1,slope1 = rect_line[1]
		# intercept2,slope2 = rect_line[2]
		# intercept3,slope3 = rect_line[3]
		
		# cv2.imshow('removed rectangle', img)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		return img

	def pnp(self, imgPoints,img):
		h,w = img.shape[:2]
		# World coordinates using window measurement in world
		long_side = 76/2
		short_side = 50/2
		objPoints = np.array([[-long_side,short_side,0],\
									[long_side,short_side,0],\
									[long_side,-short_side,0], \
									[-long_side,-short_side,0]], dtype=np.float64)
		print("obj points ",objPoints)
		print("img points", imgPoints.astype(np.float64))
		# Camera K matrix(intrinsic params)
		camMatrix = np.array([[103.97, 0 , 208.105],[0, 103.97, 114.713],[0,0,1]],dtype=np.float64)

		#distortion coefficients 
		distCoeffs = np.array([0,0,0,0,0],dtype=np.float64)
		rotVec, transVec = self.LinearPnP(objPoints,imgPoints.astype(np.float64), camMatrix, distCoeffs)
	
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
		# cv2.imshow('reprojected',img)
		# cv2.waitKey(1)
		# cv2.destroyAllWindows()

		return rotVec,transVec

	def get_outer_window_corner_points(self, corner_pts, img):
		distances = corner_pts[:,0]**2 + corner_pts[:,1]**2
		h,w = img.shape[:2]
		# print("corner_pts = ",corner_pts)
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

		addd = 7
		imgPoints = np.array([[left_top_corner[0]+addd,left_top_corner[1]+addd],
							  [right_top_corner[0]-addd,right_top_corner[1]+addd],
							  [right_bottom_corner[0]-addd,right_bottom_corner[1]-addd],
							  [left_bottom_corner[0]+addd,left_bottom_corner[1]-addd ]],dtype=np.int32)
		imgPoints = np.squeeze(imgPoints)

		return imgPoints

	def rect_mask(self,edges,allcountour_img):
		kernel = np.ones((7,7),np.uint8)
		dilated_edges = cv2.dilate(edges,kernel,iterations = 1)
		houghlines_gray, contours, hierarchy = cv2.findContours(dilated_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		epsilon = 0.1*cv2.arcLength(contours[0], True)
		approx = cv2.approxPolyDP(contours[0], epsilon, True)
		edges3CH = np.dstack((dilated_edges,dilated_edges,dilated_edges))
		# img = np.zeros_like(edges)
		corners = np.zeros((4,2))
		status = False
		if ((len(approx) >= 4)):
			points = np.squeeze(np.array(approx))
			# print("points",points)
			if cv2.isContourConvex(points):
				corners = self.get_outer_window_corner_points(points, edges3CH)
				cv2.drawContours(edges3CH, [corners], 0, (0,0,255), 3)
				status = True
				return status,corners

		status = False
		return status, corners


	def detect_ellipse_fitellipse(self,img):
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		# lines = []
		# cle = cv2.createCLAHE(clipLimit = 5.0,tileGridSize =(8,8))
		# gray = cle.apply(gray)
		# img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		# mask = ((img_hsv> np.array([0,0,230])).astype(np.float32)+(img_hsv> np.array([0,0,230])).astype(np.float32)*(-0.5) + 0.5)
		# test_img = np.uint8( (mask*img_hsv)*255/np.max(mask*img_hsv))
		# img_dark = cv2.cvtColor(test_img,cv2.COLOR_HSV2BGR)
		# gray = cv2.cvtColor(img_dark,cv2.COLOR_BGR2GRAY)
		# cv2.imshow('contrast corrected', gray)
		# cv2.waitKey(0)

		# cv2.destroyAllWindows()
		max_val = np.amax(gray)
		rand_thresh = gray.copy()
		rand_thresh[rand_thresh[:]<(max_val*self.thresh)] = 0 
		edges = cv2.Canny(rand_thresh,50,150,apertureSize = 3)
		kernel = np.ones((2,2),np.uint8)
		dilated_edges = cv2.dilate(edges,kernel,iterations = 1)
		lines = cv2.HoughLines(edges,1,np.pi/180,30)
		
		# cv2.imshow('after threshold', dilated_edges)
		# cv2.waitKey(1)
		if(lines is not None):
			# print("first hough lines shape",len(lines))
		
			i,contours,h = cv2.findContours(dilated_edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
			max_w = 0
			max_h = 0
			iter_ = -1
			count = 0
			# print("contours",np.shape(contours))
			for i,c1 in enumerate(contours):
				# print i
				# print("c1 = ", c1)
				(x,y,we,he) = cv2.boundingRect(c1)
				# print("bb params = ",x,y,we,he)

				if len(c1)>4 and (we > 10 and he > 10):
						if he>max_h or we>max_w:
							max_h = he
							max_w = we
							iter_=i
						count+=1
			stencil = np.zeros(edges.shape).astype(edges.dtype)
			color = [255, 255, 255]
			cv2.fillPoly(stencil, contours[iter_], color)
			hough_status, rect_corners = self.rect_mask(stencil,edges)
			# hough_status,houg_lines = self.get_hough_lines(stencil)
			if(hough_status):
				try:
					rot,trans = self.pnp(rect_corners,img)
					# print("transVec shape = ", trans.shape,trans)
					# trans = np.reshape(trans,(3,1))
					# print("centers shape = ", self.centers.shape)

					if(self.centers.shape[0]<self.filter_len):
						self.centers = np.vstack((self.centers,np.array([trans[0,0],trans[1,0]])))
					else:
						self.centers = np.vstack((self.centers,np.array([trans[0,0],trans[1,0]])))
						self.centers = np.delete(self.centers,0,0)

					center_mean= np.mean(self.centers,axis = 0)
					self.pose_obj.position.x = center_mean[0]			#in m
					self.pose_obj.position.y = center_mean[1] 	#in m
					# self.pose_obj.position.z = trans[2,0]  	#in m
					# self.pose_obj.orientation.x = 0
					# self.pose_obj.orientation.y = 0
					# self.pose_obj.orientation.z = 0

					# print("state x,y,z",self.pose_obj.position.x,self.pose_obj.position.y,self.pose_obj.position.z)
					# self.pose_pub.publish(self.pose_obj)
					# cv2.imshow("warped img",im_out)
					# cv2.waitKey(0)
					# cv2.destroyAllWindows()

					# print("I was able to detect outer rectangle")
				except:
					pass
			else:
				print("outer rectangle not detected")


				
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
		# cv2.imshow('edges', rand_thresh)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		# cv2.imshow('eroded', eroded_img)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		img_three = eroded_img.copy()
		img_three = np.dstack((img_three,eroded_img,eroded_img))
		# print("3d img shape = ",img_three.shape)
		circles = cv2.HoughCircles(eroded_img,cv2.HOUGH_GRADIENT,1,10,param1=150,param2=50,minRadius=0,maxRadius=0)
		# print("circles = ",circles)
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
			# cv2.imshow('circles', img_three)
			# cv2.waitKey(0)
			# cv2.destroyAllWindows()
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
		if self.detect_flag == True:
			if self.image is not None:
				# cv2.imshow('cb', self.image)
				# cv2.waitKey(1)
				self.detect_ellipse_fitellipse(self.image)
				print("detecting")

			else:
				print("No image published")
			center_mean= np.mean(self.centers,axis = 0)
			self.pose_obj.position.x = (center_mean[0]/100) + 0.5			#in m
			self.pose_obj.position.y = center_mean[1]/100
			self.pose_pub.publish(self.pose_obj)
		else:
			print("Node on stand")


def main():
		rospy.init_node('image_reader', anonymous=True)
		ob = BullsEyeDetection()
		rate = rospy.Rate(15)
		while(not rospy.is_shutdown()):
			ob.run_pipeline()
			rate.sleep()


if __name__ == '__main__':
		main()
