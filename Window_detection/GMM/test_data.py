import numpy as np
import cv2
import copy
import sys
import matplotlib.pyplot as plt
import math
import os
import imutils
from imutils import contours


def gaussian(data,mean,cov):
	det_cov = np.linalg.det(cov)
	cov_inv = np.linalg.inv(cov)
	diff = np.matrix(data-mean)
		
		
	N = (2.0 * np.pi) ** (-len(data[1]) / 2.0) * (1.0 / (np.linalg.det(cov) ** 0.5)) *\
			np.exp(-0.5 * np.sum(np.multiply(diff*cov_inv,diff),axis=1))
		
	return N
		
		
def test_combined(test_image,K,n_factor,weights, parameters,color):

	    		
	nx = test_image.shape[0]
	ny = test_image.shape[1]
	
	img = test_image
	ch = img.shape[2]
	img = np.reshape(img, (nx*ny,ch))
	
	prob = np.zeros((nx*ny,K))
	
	likelihood = np.zeros((nx*ny,K))
	
	for cluster in range(K):
	   prob[:,cluster:cluster+1] = weights[cluster]*gaussian(img,parameters[cluster]['mean'], parameters[cluster]['cov'])
	   
	   likelihood = prob.sum(1)
	   
	
	probabilities = np.reshape(likelihood,(nx,ny))
	
	probabilities[probabilities>np.max(probabilities)/n_factor] = 255
	
	output = np.zeros_like(frame)
	
	output[:,:,0] = probabilities
	output[:,:,1] = probabilities
	output[:,:,2] = probabilities
	
	blur = cv2.GaussianBlur(output,(3,3),5)
	cv2.imshow("out",output)
	
	edged = cv2.Canny(blur,50,255 )
	
	cnts,h = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	(cnts_sorted, boundingBoxes) = contours.sort_contours(cnts, method="left-to-right")
	
	hull = cv2.convexHull(cnts_sorted[0])
	(x,y),radius = cv2.minEnclosingCircle(hull)
	
	if radius > 7:
		cv2.circle(test_image,(int(x),int(y)-1),int(radius+1),(color),4)
	return test_image	
			
	
if __name__ == "__main__":
	name = "detectbuoy.avi"
	video = []
	cap = cv2.VideoCapture(name)
	while (cap.isOpened()):
		success, frame = cap.read()
		if success == False:
			break

		K = 4
		n = 8.5

		test_image = frame
		weights = np.load('green_weights.npy')
		parameters = np.load('params_green.npy')
		test_combined(test_image, K,n, weights, parameters,(0,255,0))
		
		K = 6
		n = 3.0
		weights = np.load('orange_weights.npy')
		parameters = np.load('params_orange.npy')
		test_combined(test_image, K,n,weights, parameters,(0,165,255))

		K = 7
		N = 9.5

		weights = np.load('yellow_weights.npy')
		parameters = np.load('params_yellow.npy')
		test_combined(test_image, K,n,weights, parameters,(0,255,255))
		video.append(test_image)
		# cv2.waitKey(1)
		cv2.imshow("Final output",test_image)
		cv2.waitKey(1)


	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = cv2.VideoWriter('multi_all.avi', fourcc, 5.0, (640, 480))
	for v in video:
		out.write(v)
		cv2.waitKey(1)


	out.release()
		
	cap.release()
			
	
	
	
	
	
	
	