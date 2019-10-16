import logging
import os
import cv2
import numpy as np
from matplotlib import pyplot as plt

from roipoly import MultiRoi,RoiPoly

# def yes_or_no(question):
#     answer = input(question + "(y/n): ").lower().strip()
#     print("")
#     print(answer)
#     while not(answer == "y" or answer == "yes" or \
#     answer == "n" or answer == "no"):
#         print("Input yes or no")
#         answer = input(question + "(y/n):").lower().strip()
#         print("")
#     if answer[0] == 'n':
#         return True
#     else:
#         return False



def main():

    # logging.basicConfig(format='%(levelname)s ''%(processName)-10s : %(asctime)s ','%(module)s.%(funcName)s:%(lineno)s %(message)s',level=logging.INFO)

    # load images
    data_path = "/home/pratique/drone_course_data/window_detection/windowPhotos"
    reply = 'k'

    for filename in os.listdir(data_path):

        img = cv2.imread(os.path.join(data_path,filename))
        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
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

        # Show ROI masks
        plt.imshow(np.bitwise_not(roi2.get_mask(img) ^ roi1.get_mask(img)),
                   interpolation='nearest', cmap="Greys")
        plt.title('ROI masks of the two ROIs')
        plt.show()
        
        # if yes_or_no("wanna continue?"):
        #     continue
        # else:
        #     break



if __name__ == '__main__':
    main()