import logging
import os
import cv2
import numpy as np
from matplotlib import pyplot as plt

from roipoly import MultiRoi,RoiPoly

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

    # logging.basicConfig(format='%(levelname)s ''%(processName)-10s : %(asctime)s ','%(module)s.%(funcName)s:%(lineno)s %(message)s',level=logging.INFO)

    # load images
    data_path  = '/home/abhinav/Gits/drone-course/drone_course_data/Window_detection/GMM/data/GMM_1/'
    reply = 'k'
    i = 0
    for filename in os.listdir(data_path):
        # print filename
        if i%10 != 0:
            i+=1
            continue
        img = cv2.imread(os.path.join(data_path,filename))
        convert = tune_RGB(img)
    	convert = tune_HSV(convert)
    	convert = adjust_gamma(convert, gamma = 1.5)
    	convert = cv2.fastNlMeansDenoisingColored(convert, None, 3, 3, 7, 15)
        img = cv2.cvtColor(convert,cv2.COLOR_BGR2RGB)
        # Show the image
        # Show the image
        fig = plt.figure()
        plt.imshow(img, interpolation='nearest', cmap="Greys")
        plt.colorbar()
        plt.title("left click: line segment         right click or double click: close region")
        plt.show(block=False)

        # Let user draw first ROI
        roi1 = RoiPoly(color='r', fig=fig)

        # Show the image with the first ROI
        fig = plt.figure()
        plt.imshow(img, interpolation='nearest', cmap="Greys")
        plt.colorbar()
        roi1.display_roi()
        plt.title('draw second ROI')
        plt.show(block=False)

        # Let user draw second ROI
        roi2 = RoiPoly(color='b', fig=fig)

        # Show the image with both ROIs and their mean values
        plt.imshow(img, interpolation='nearest', cmap="Greys")
        plt.colorbar()
        for roi in [roi1, roi2]:
            roi.display_roi()
            roi.display_mean(img)
        plt.title('The two ROIs')
        plt.show()

        # plt.imshow(np.bitwise_not(roi2.get_mask(img) ^ roi1.get_mask(img)),
        #            interpolation='nearest', cmap="Greys")
        # plt.title('ROI masks of the two ROIs')
        # plt.show()

        #store data
        mask = roi2.get_mask(img) ^ roi1.get_mask(img)
        mask.astype(int)
        # print(np.uint8(mask))
        window_mask = np.uint8(mask*255) 
        extracted_frame = cv2.bitwise_and(img, img, mask = window_mask)
        extracted_frame = cv2.cvtColor(extracted_frame,cv2.COLOR_RGB2BGR)
        # input('aa')
        cv2.imwrite("./data/window"+str(i)+".jpg", extracted_frame)
        i = i+1
        if 0xFF == ord('q'):
            break



if __name__ == '__main__':
    main()