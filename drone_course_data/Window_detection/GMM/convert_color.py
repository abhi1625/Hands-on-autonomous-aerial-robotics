import cv2
import numpy as np

def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
 
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)

def main():
	data_path  = "/home/pratique/drone-course/Window_detection/GMM/data/GMM_1/frame0000.jpg"
	img = cv2.imread(data_path)

	convert = np.float32(img)
	convert[:,:,1] = convert[:,:,1] - 40*np.ones_like(img[:,:,1])
	convert[:,:,1] = convert[:,:,0] + 10*np.ones_like(img[:,:,0])

	convert[convert>255]=255
	convert[convert<0] = 0


	convert = np.uint8(convert)
	convert = adjust_gamma(convert, gamma = 1.5)
	cv2.imshow("img",convert)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()