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

    def run_pipeline(self):
		dir_list_left = sorted(os.listdir(self.data_path+'Helix_left'))
		dir_list_right = sorted(os.listdir(self.data_path+'Helix_right'))
		prev_img_left = None
		surf = cv2.xfeatures2d.SURF_create(hessianThreshold = 900)
		feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

		# Parameters for lucas kanade optical flow
		lk_params = dict( winSize  = (15,15),
		                  maxLevel = 2,
		                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

		# Create some random colors
		color = np.random.randint(0,255,(100,3))

		ind = 0
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
				ind+=1

				# Extract SURF features
				# img2 = cv2.drawKeypoints(prev_img_left,p0,None,(255,0,0),4)
				if ((ind == 1 or ind%self.detect_corner_reset_num == 0) and ind!=0):
					# print("corner detected at frame number {}".format(ind))
					
					# kp, des = surf.detectAndCompute(prev_img_left, None)
					# print("type kp: {}, kp shape: {}".format(type(kp),des.shape))

					prev_gray = cv2.cvtColor(prev_img_left, cv2.COLOR_BGR2GRAY)
					prev_corners = cv2.goodFeaturesToTrack(prev_gray, mask = None, **feature_params)
					# print("type gdftTT corner: {}, and shape: {}".format(type(prev_corners),prev_corners.shape))

						# Create a mask image for drawing purposes
					mask = np.zeros_like(prev_img_left)

				# VISUALIZE CORNERS FROM goodFeaturesToTrack
				# prev_corners = np.int0(prev_corners)
				# for i in prev_corners:
				# 	x,y = i.ravel()
				# 	cv2.circle(prev_img_left,(x,y),3,255,-1)
				# cv2.imshow("right",prev_img_left)
				# cv2.waitKey(0)
				# cv2.destroyAllWindows()
				draw_img_left = img_left.copy()
				curr_frame_gray = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)

				# calculate optical flow
				curr_corners, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, curr_frame_gray, prev_corners, None, **lk_params)

				# Select good points
				good_curr = curr_corners[st==1]
				good_prev = prev_corners[st==1]

				# draw the tracks
				for i,(new,old) in enumerate(zip(good_curr, good_prev)):
				    a,b = new.ravel()
				    c,d = old.ravel()
				    mask = cv2.arrowedLine(mask, (c,d), (a,b), color[i].tolist(), 2, tipLength = 0.2)
				    frame = cv2.circle(draw_img_left,(a,b),5,color[i].tolist(),-1)
				img = cv2.add(frame,mask)

				cv2.imshow('frame',img)
				k = cv2.waitKey(70) & 0xff
				if k == 27:
				    break

				# Now update the previous frame and previous points
				prev_gray = curr_frame_gray.copy()
				prev_corners = good_curr.reshape(-1,1,2)
			prev_img_left = img_left.copy()

			   

def main():
    StereoVO_obj = StereoVO()
    StereoVO_obj.run_pipeline()

if __name__ == '__main__':
    main()

