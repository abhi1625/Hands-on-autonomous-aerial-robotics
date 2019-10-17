import cv2
import numpy as np
import os

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

def main():
	data_path  = '/home/abhinav/Gits/drone-course/drone_course_data/Window_detection/GMM/data/GMM_1/frame0000.jpg'
	if (os.path.isfile(data_path)):
		print('image is present')
	else:
		print "no image"
	convert = cv2.imread(data_path)
	# convert = cv2.cvtColor(convert, cv2.COLOR_BAYER_GR2RGB)
	# convert = tune_LAB(convert)
	convert = tune_RGB(convert)
	convert = tune_HSV(convert)
	convert = adjust_gamma(convert, gamma = 1.5)
	convert = cv2.fastNlMeansDenoisingColored(convert, None, 3, 3, 7, 15)
	cv2.imshow("img",convert)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()