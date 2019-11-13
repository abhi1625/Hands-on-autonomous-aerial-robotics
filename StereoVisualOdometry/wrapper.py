#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
import numpy as np
import math
import os
class StereoVO:
	def __init__(self):
		self.data_path = "/home/pratique/drone_course_data/StereoVisualOdometry/"
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

	def get_optical_flow(self,curr_frame_3D, prev_frame_gray, prev_corners, mask, counter):
		counter+=1

		# Extract SURF features
		# img2 = cv2.drawKeypoints(prev_img_left,p0,None,(255,0,0),4)
		if ((counter == 1 or counter%self.detect_corner_reset_num == 0) and counter!=0):
			print("corner detected at frame number {}".format(counter))
			
			# kp, des = surf.detectAndCompute(prev_img_left, None)
			# print("type kp: {}, kp shape: {}".format(type(kp),des.shape))

			# prev_frame_gray = cv2.cvtColor(prev_frame_3D, cv2.COLOR_BGR2GRAY)
			prev_corners = cv2.goodFeaturesToTrack(prev_frame_gray, mask = None, **self.feature_params)
			# print("type gdftTT corner: {}, and shape: {}".format(type(prev_corners),prev_corners.shape))

				# Create a mask image for drawing purposes
			mask = np.zeros_like(curr_frame_3D)

		# VISUALIZE CORNERS FROM goodFeaturesToTrack
		# prev_corners = np.int0(prev_corners)
		# for i in prev_corners:
		# 	x,y = i.ravel()
		# 	cv2.circle(prev_img_left,(x,y),3,255,-1)
		# cv2.imshow("right",prev_img_left)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()
		# draw_img_left = curr_frame_3D.copy()
		curr_frame_gray = cv2.cvtColor(curr_frame_3D, cv2.COLOR_BGR2GRAY)

		# calculate optical flow
		curr_corners, st, _ = cv2.calcOpticalFlowPyrLK(prev_frame_gray, curr_frame_gray, prev_corners, None, **self.lk_params)

		# Select good points
		good_curr = curr_corners[st==1]
		good_prev = prev_corners[st==1]

		# draw the tracks
		# for i,(new,old) in enumerate(zip(good_curr, good_prev)):
		#     a,b = new.ravel()
		#     c,d = old.ravel()
		#     mask = cv2.arrowedLine(mask, (c,d), (a,b), self.color[i].tolist(), 2, tipLength = 0.2)
		#     frame = cv2.circle(draw_img_left,(a,b),5,self.color[i].tolist(),-1)

		# img = cv2.add(frame,mask)
		# show optical flow vectors superimposed on image
		# cv2.imshow('frame',img)
		# cv2.waitKey(70)
		# k = cv2.waitKey(70) & 0xff
		# if k == 27:
		#     break
		# print("good curr shape {} and good prev shape {}".format(good_curr.shape, good_prev.shape))
		Optical_flow = good_curr-good_prev
		# print("optical flow = {} , good curr {} , good prev {}".format(Optical_flow[:3,:],good_curr[:3,:],good_prev[:3,:]))

		# Now update the previous frame and previous points
		prev_frame_gray = curr_frame_gray.copy()
		prev_corners = good_curr.reshape(-1,1,2)

		return Optical_flow, prev_frame_gray, prev_corners, mask,counter


	def get_depth(self, img_left, img_right):
		img_left_gray = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
		img_right_gray = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
		disparity = self.stereo.compute(img_left_gray,img_right_gray)
		disparity = np.float32(disparity)
		disparity[disparity == 0.0] = 0.0001
		# disparity3d = np.dstack((disparity,disparity,disparity))
		# print('disparity max and min',np.amax(disparity),np.amin(disparity), np.mean(np.mean(disparity)))
		print("mean disparity = {}".format(np.mean(np.mean(disparity))))
		depth_map = np.reciprocal(disparity)
		print("mean depth = {}".format(np.mean(np.mean(depth_map))))
		cv2.imshow("disparity",depth_map)
		cv2.waitKey(70)
		return disparity

	def run_pipeline(self):
		dir_list_left = sorted(os.listdir(self.data_path+'Helix_left'))
		# dir_list_right = sorted(os.listdir(self.data_path+'Helix_right'))
		prev_img_left, prev_frame, prev_corners, mask = (None,)*4

		counter = 0
		for filename in dir_list_left:
			left_img_path = os.path.join(self.data_path,'Helix_left',filename)
			right_img_path = os.path.join(self.data_path,'Helix_right',filename)
			# print("left img path = ",left_img_path)
			#print("right img path = =",right_img_path)
			img_left = cv2.imread(left_img_path)
			img_right = cv2.imread(right_img_path)
			# cv2.imshow("left",img_left)
			# cv2.waitKey(0)
			# cv2.destroyAllWindows()
			# cv2.imshow("right",img_right)
			# cv2.waitKey(0)
			# cv2.destroyAllWindows()


			if prev_img_left is not None:
				prev_frame = cv2.cvtColor(prev_img_left,cv2.COLOR_BGR2GRAY)
				Optical_flow, prev_frame, prev_corners, mask, counter = self.get_optical_flow(img_left, prev_frame, prev_corners, mask, counter)
				disparity_map = self.get_depth(img_left, img_right)

				
			# update previous image
			prev_img_left = img_left

			   

def main():
    StereoVO_obj = StereoVO()
    StereoVO_obj.run_pipeline()

if __name__ == '__main__':
    main()

