#!/usr/bin/env python
import time
import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist,Pose
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
# from GMM.test_data import *

class video_stream:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/image_raw", Image, self.img_callback)
		self.wall_detection = Wall_detection()
		rospack = rospkg.RosPack()
		self.image_cv = None
        	self.prev_img = None
        	self.ind = 0
		self.pose_pub = rospy.Publisher("/relative_pose", Pose, queue_size=100)
		self.rel_pose = Pose()
        	self.mean_pose = Pose()
	def img_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			print('got frame')
		except CvBridgeError as e:
			print(e)
		
		if cv_image is not None:
            		if self.prev_img is not None:
                		# processed_img = preprocess_img(cv_image)
                		h_error, v_error = self.wall_detection.compute_error(self.prev_img, cv_image, self.ind)
                		#print h_error, v_eirror
                        self.rel_pose.position.x = 0.0
                        self.rel_pose.position.y = h_error/640
                        self.rel_pose.position.z = 0.0
                        self.rel_pose.orientation.z = h_error/640

                	self.prev_img = cv_image
                	self.ind += 1
		self.pose_pub.publish(self.rel_pose)

class Wall_detection():
    def __init__(self):
        self.bridge = CvBridge()
        self.w = 1280
        self.h = 720


    def detect_blobs(self,im):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
                
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 255
        params.blobColor = 255
        params.filterByColor = True
            
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 2500
        params.maxArea = 10000000000000
            
        # # Filter by Circularity
        params.filterByCircularity = False
        # params.minCircularity = 0.1
                
        # # Filter by Convexity
        params.filterByConvexity = False
        # params.minConvexity = 0.9
            
        # # Filter by Inertia
        params.filterByInertia = False
        # params.minInertiaRatio = 0.1
            
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)

        keypoints = detector.detect(im)
        # if keypoints:
        #     print("keypoints = ",keypoints[0].pt)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob

        # im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # # Show blobs
        # cv2.imshow("Keypoints", im_with_keypoints)
        # cv2.waitKey(1)
        # cv2.destroyAllWindows()
        return keypoints

    def get_wall_center(self,cv_image):
        cv_image = np.dstack((cv_image, cv_image, cv_image))
        kernel = np.ones(30)
        h = cv_image.shape[0]
        w = cv_image.shape[1]
        closing = cv2.morphologyEx(cv_image, cv2.MORPH_CLOSE, kernel)
        thresh = np.mean(closing[:])
        closing[closing < thresh] = 0
        closing_resize = cv2.resize(closing, (w/4, h/4))
        Z = closing_resize.reshape((-1,3))
            # convert to np.float32
        Z = np.float32(Z)
            # define criteria, number of clusters(K) and apply kmeans()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 4
        ret,label,center=cv2.kmeans(Z,K,None,criteria,5,cv2.KMEANS_RANDOM_CENTERS)
        print(np.amax(label), np.amin(label), center.shape)
        # Now convert back into uint8, and make original image
        center = np.uint8(center)
        res = center[label.flatten()]
        max_center = np.amax(res)
        print(max_center)
        res[res<max_center] = 0
        res[res>=max_center] = 255
        res2 = res.reshape((h/4,w/4,3))
        closing = cv2.resize(res2,(w,h))
        closing = np.uint8(closing)
        keypoints = self.detect_blobs(closing)
        print("length",len(keypoints))
        if len(keypoints) >= 1:
            center_x = np.mean(np.array(keypoints[0].pt[0]))
            center_y = np.mean(np.array(keypoints[0].pt[1]))
            cv2.circle(cv_image,(int(center_x),int(center_y)),int(keypoints[0].size/2),(255,0,255),2)
        #cv2.imshow("test_img",cv_image)
        #cv2.waitKey(1)
        return keypoints

    def compute_error(self,prev_frame,curr_frame,ind):
        h_error = -640
        v_error = -360
        img_left_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        img_right_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
        hsv = np.zeros_like(prev_frame[:,:,0])
        # hsv = np.zeros_like(prev_frame)
        # hsv[...,1] = 255
        flow = cv2.calcOpticalFlowFarneback(img_left_gray,img_right_gray, None, 0.5, 3, 30, 3, 5, 1.5, 0)
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        # hsv[...,0] = ang*180/np.pi/2
        hsv = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        flow_img = np.uint8(hsv)
        # self.tri_flow.append(flow_img)
        # if len(self.tri_flow) >3:
        # 	del self.tri_flow[0]
        # mean_flow = np.mean(np.array(self.tri_flow), axis=0)
        # print("mean_flow_shape: ",mean_flow.shape)
        # mean_flow = cv2.normalize(mean_flow,None,0,255,cv2.NORM_MINMAX)
        # mean_flow = np.uint8(mean_flow)
        #flow_msg = self.bridge.cv2_to_imgmsg(flow_img, "mono8")
        #curr_msg = self.bridge.cv2_to_imgmsg(curr_frame, "bgr8")
        #self.flow_pub.publish(flow_msg)
        #self.curr_frame_pub.publish(curr_msg)
        keypoints = self.get_wall_center(flow_img)
        if len(keypoints) >=1:
            h_error = (self.w/2) - float(keypoints[0].pt[0])
            v_error = (self.h/2) - float(keypoints[0].pt[1])
        return h_error, v_error

def main():
	count = 0
	ob = video_stream()

	rospy.init_node('image_reader', anonymous=True)
	rate = rospy.Rate(30)
	while(not rospy.is_shutdown()):
		# rospy.spin()
		# ob.processing()
		count += 1;
		print(count)
		rate.sleep()

if __name__ == '__main__':
	main()
