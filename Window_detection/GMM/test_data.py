import numpy as np
import cv2
import copy
import sys
import matplotlib.pyplot as plt
import math
import os
# import imutils
# from imutils import contours

def tune_HSV(img):
	img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# print img.shape
	convert = np.float32(img)
	# print convert.shape
	#red channel
	convert[:,:,2] = 1.1*convert[:,:,2]
	#green channel
	convert[:,:,1] = (512/512)*(convert[:,:,1] + 40*np.ones_like(img[:,:,1]))
	#blue channel
	convert[:,:,0] = (512/512)*(convert[:,:,0] + 10*np.ones_like(img[:,:,0]))
	convert[convert>255]=255
	convert[convert<0] = 0

	convert = np.uint8(convert)
	convert = cv2.cvtColor(convert, cv2.COLOR_HSV2BGR)
	return convert

def tune_LAB(img):
	img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
	# print img.shape
	convert = np.float32(img)
	# print convert.shape
	#red channel
	# convert[:,:,2] = 0.5*convert[:,:,2]
	#green channel
	convert[:,:,1] = (512/512)*(convert[:,:,1] + 20*np.ones_like(img[:,:,1]))
	#blue channel
	convert[:,:,2] = (512/512)*(convert[:,:,2] - 20*np.ones_like(img[:,:,0]))
	convert[convert>255]=255
	convert[convert<0] = 0

	convert = np.uint8(convert)
	convert = cv2.cvtColor(convert, cv2.COLOR_LAB2BGR)
	return convert
def tune_RGB(img):
	# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	# print img.shape
	convert = np.float32(img)
	# print convert.shape
	#red channel
	convert[:,:,0] = (512/512)*(convert[:,:,0] + 20*np.ones_like(img[:,:,0]))
	#green channel
	convert[:,:,1] = (512/512)*(convert[:,:,1] - 40*np.ones_like(img[:,:,1]))
	#blue channel
	convert[:,:,2] = (512/512)*(convert[:,:,2] + 20*np.ones_like(img[:,:,0]))
	convert[convert>255]=255
	convert[convert<0] = 0

	convert = np.uint8(convert)
	# convert = cv2.cvtColor(convert, cv2.COLOR_RGB2BGR)
	return convert

def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
 
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)


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
	frame = test_image[:,:,:]
	img = test_image
	ch = img.shape[2]
	img = np.reshape(img, (nx*ny,ch))
	
	prob = np.zeros((nx*ny,K))
	
	likelihood = np.zeros((nx*ny,K))
	
	for cluster in range(K):
	   prob[:,cluster:cluster+1] = weights[cluster]*gaussian(img,parameters[cluster]['mean'], parameters[cluster]['cov'])
	   
	   likelihood = prob.sum(1)
	   
	
	probabilities = np.reshape(likelihood,(nx,ny))
	test = np.uint8(probabilities*255/np.max(probabilities))
	
	probabilities[probabilities>np.mean(probabilities)/n_factor] = 255
	plt.imshow(probabilities)
	plt.show()
	
	# output = np.zeros_like(frame)
	# # print output.shape
	
	# output[:,:,0] = probabilities
	# output[:,:,1] = probabilities
	# output[:,:,2] = probabilities
	
	# blur = cv2.GaussianBlur(output,(3,3),5)
	# print blur
	# cv2.imshow("out",output)
	# input('a')
	# edged = cv2.Canny(blur,50,255 )
	
	# cnts,h = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	# (cnts_sorted, boundingBoxes) = contours.sort_contours(cnts, method="left-to-right")
	
	# hull = cv2.convexHull(cnts_sorted[0])
	# (x,y),radius = cv2.minEnclosingCircle(hull)
	
	# if radius > 7:
	# 	cv2.circle(test_image,(int(x),int(y)-1),int(radius+1),(color),4)
	return probabilities	
			
	
if __name__ == "__main__":
	K = 2
	n = 0.1
	test_image_path  = '/home/abhinav/Gits/drone-course/drone_course_data/Window_detection/GMM/data/GMM_1/frame0010.jpg'
	test_image = cv2.imread(test_image_path)
	convert = tune_RGB(test_image)
	convert = tune_HSV(convert)
	convert = adjust_gamma(convert, gamma = 1.0)
	convert = cv2.fastNlMeansDenoisingColored(convert, None, 3, 3, 7, 15)
	test_image = cv2.cvtColor(convert, cv2.COLOR_BGR2RGB)

	weights = np.load('./training_params/window_weights.npy', allow_pickle=True)
	parameters = np.load('./training_params/gaussian_params.npy', allow_pickle=True)
	print test_image.shape
	save_img = test_combined(test_image, K,n,weights, parameters,(0,165,255))
	cv2.imwrite("test_image.jpg", save_img)
# if __name__ == "__main__":
# 	name = "detectbuoy.avi"
# 	video = []
# 	cap = cv2.VideoCapture(name)
# 	while (cap.isOpened()):
# 		success, frame = cap.read()
# 		if success == False:
# 			break

# 		K = 4
# 		n = 8.5

# 		test_image = frame
# 		weights = np.load('green_weights.npy')
# 		parameters = np.load('params_green.npy')
# 		test_combined(test_image, K,n, weights, parameters,(0,255,0))
		
# 		K = 6
# 		n = 3.0
# 		weights = np.load('orange_weights.npy')
# 		parameters = np.load('params_orange.npy')
# 		test_combined(test_image, K,n,weights, parameters,(0,165,255))

# 		K = 7
# 		N = 9.5

# 		weights = np.load('yellow_weights.npy')
# 		parameters = np.load('params_yellow.npy')
# 		test_combined(test_image, K,n,weights, parameters,(0,255,255))
# 		video.append(test_image)
# 		# cv2.waitKey(1)
# 		cv2.imshow("Final output",test_image)
# 		cv2.waitKey(1)


# 	fourcc = cv2.VideoWriter_fourcc(*'XVID')
# 	out = cv2.VideoWriter('multi_all.avi', fourcc, 5.0, (640, 480))
# 	for v in video:
# 		out.write(v)
# 		cv2.waitKey(1)


# 	out.release()
		
# 	cap.release()
			
	
	
	
	
	
	
	