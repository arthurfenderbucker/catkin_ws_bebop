#!/usr/bin/env python
import cv2
import numpy as np

colors = np.random.randint(0, 255, (30, 3))
def crop_rect( img, rect):
    # get the parameter of the small rectangle
    center, size, angle = rect[0], rect[1], rect[2]
    center, size = tuple(map(int, center)), tuple(map(int, size))

    # get row and col num in img
    height, width = img.shape[0], img.shape[1]

    # calculate the rotation matrix
    M = cv2.getRotationMatrix2D(center, angle, 1)
    # rotate the original image
    img_rot = cv2.warpAffine(img, M, (width, height))

    # now rotated rectangle becomes vertical and we crop it
    img_crop = cv2.getRectSubPix(img_rot, size, center)

    return img_crop, img_rot

def filter_cnts( cnts_raw, filter_rotation = False, min_sides = 4, max_sides = 4, max_perpendicular_factor = 30, min_area = 2000):
    cnts = []
    # loop over our contours
    for c in cnts_raw:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        # if our approximated contour has four points, then
        # we can assume that we have found our screen
        if len(approx) >= min_sides and len(approx) <= max_sides and perpendicular(approx) < max_perpendicular_factor and cv2.contourArea(c)>min_area:
            center, size, angle = cv2.minAreaRect(approx)
            
            if angle < -75 or angle > -15 or not filter_rotation:
                cnts.append(approx)
    
    return np.array(cnts)

def show_poligons(image, poligons, window_name = "window"):
    try:
        for i in range(len(poligons)):
            cv2.drawContours(
                image, [poligons[i]], -1, colors[i].tolist(), 3)
        cv2.imshow(window_name, image)
        return 1
    except:
        print("erros")
        return 0
def show_rects(image, rects):
    if not rects == None and len(rects)>0:
        # print(tuple(rects))
        rects = np.int0(rects)
        for r in rects:
            cv2.rectangle(image,tuple(r[0,:]),tuple(r[1,:]),(0,0,255),2)
        # return image
    return image

def area_and_perp( points):
    print(cv2.contourArea(points))
    if cv2.contourArea(points) > 0:
        return cv2.contourArea(points)
    else:
        return 0

def perpendicular( points):
    total_ang = 0
    for i in range(len(points)):
        l = i - 2
        m = i - 1
        if i == 0:
            l = len(points) - 2
            m = len(points) - 1
        elif i == 1:
            l = len(points) - 1
        l1 = points[i][0] - points[m][0]
        l2 = points[m][0] - points[l][0]

        total_ang += abs(ang(l1, l2))
    return total_ang / len(points)

def ang( a, b):
    def norm(x):
        mag = np.linalg.norm(x)
        return x / mag
    return np.degrees(np.dot(norm(a), norm(b)))