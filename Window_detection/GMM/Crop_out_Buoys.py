import cv2
import numpy as np


def catch(pt):
    img1 = cv2.imread('frame29.jpg')
    k = pt
    p = list()
    print(k)
    for elem in k:
        elem = list(elem)
        p.append(elem)
    print(p)

    p = np.array(p)

    mask = np.zeros_like(img1)
    cv2.drawContours(mask,  [p], -1, [255, 255, 255], -1)
    mask = cv2.bitwise_not(mask)
    final = cv2.add(img1, mask)
    cv2.imshow('new', img1)
    cv2.imshow('white', mask)
    cv2.imshow('final', final)

    x, y, c = np.where(final != 255)

    top_left_x, top_left_y = (np.min(x), np.min(y))
    bottom_right_x, bottom_right_y = (np.max(x), np.max(y))

    top_left_x = top_left_x - 10
    top_left_y = top_left_y - 10

    bottom_right_x = bottom_right_x + 10
    bottom_right_y = bottom_right_y + 10

    crop_image = final[top_left_x:bottom_right_x, top_left_y:bottom_right_y]

    cv2.imshow('crop', crop_image)
    cv2.imwrite("Green_boi_29.jpg", crop_image)
    cv2.waitKey(0)


def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Place a point on the location of click
        cv2.circle(img, (x, y), 1, (0, 0, 255), -1)
        point.append((x, y))

        # draw a line on the image once we have 2 or more points in point list
        if len(point) >= 2:
            cv2.line(img, point[-1], point[-2], (0, 0, 0), 2)
        cv2.imshow('original', img)
    if event == cv2.EVENT_RBUTTONDOWN:
        catch(point)


img = cv2.imread('frame29.jpg')
cv2.imshow('original', img)
point = []
cv2.setMouseCallback("original", click_event)

cv2.waitKey(0)
