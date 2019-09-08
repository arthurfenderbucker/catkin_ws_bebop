#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
import rospkg

from cv_bridge import CvBridge, CvBridgeError



from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import matplotlib.pyplot as plt

import json

rospack = rospkg.RosPack()
config_path = str(rospack.get_path('simulation')+'/config/color_config.json')


class color_detection():
    colors = np.random.randint(0, 255, (10, 3))
    color_tracking = True
    def __init__(self,color="blue"):
        #get min and max HSV values from config file
        self.color = color
        with open(config_path) as json_data_file:
            self.config_data = json.load(json_data_file)
        print(self.config_data)
        if color in self.config_data.keys():
            print("existing color selected!")     
        else:
            print("new color selected!")
            self.config_data[color] = {"min":{"H":0,"S":0,"V":0}, "max":{"H":255,"S":255,"V":255}}

        self.min_values = self.config_data[color]["min"].values()
        self.max_values = self.config_data[color]["max"].values()
        self.calibrate()
    
    def crop_rect(self, img, rect):
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
    def nothing(self,x):
        pass
    def get_rect(self,image, show = True):
        
        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert to HSV
        # cv2.imshow("Trackbars",self.color_range_image)
        thresh = cv2.inRange(frame_to_thresh,tuple(self.min_values), tuple(self.max_values))

        # ignore random noise and merge very close areas
        kernel = np.ones((5,5),np.uint8) 
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # print(cv2.mean(image,mask))
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            rect = cv2.minAreaRect(c)
            img_crop, img_rot = self.crop_rect(image, rect)
            
            # print(rect)  
            if show:
                cv2.imshow("rect",img_crop)
            # print(rect)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if show:
                cv2.drawContours(image,[box],0,(0,0,255),2)

            # only proceed if the radius meets a minimum size
            if show and radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(image, center, 3, (0, 0, 255), -1)
                cv2.putText(image,"centroid", (center[0]+10,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
                cv2.putText(image,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)

            
            # show the frame to our screen
        
            if show :
                cv2.imshow("Original", image)
                cv2.imshow("Thresh", thresh)
                cv2.imshow("Mask", mask)
            return [center,radius,rect]
        if show :
            cv2.imshow("Original", image)
            cv2.imshow("Thresh", thresh)
            cv2.imshow("Mask", mask)
        return [None, None, None]

    def save_color(self):
        with open(config_path, 'w') as json_data_file:
            json.dump(self.config_data, json_data_file)
        print("changes saved!")

    def calibrate(self):
        self.setup_trackbars()
        # pass
    def tracker_callback(self,x):
        for k,j in enumerate("HSV"):
            self.min_values[k] = cv2.getTrackbarPos("min_%s" % (j), "Trackbars")
            self.config_data[self.color]["min"][j] = self.min_values[k]

            self.max_values[k] = cv2.getTrackbarPos("max_%s" % (j), "Trackbars")
            self.config_data[self.color]["max"][j] = self.max_values[k]
        

    def setup_trackbars(self):
        cv2.namedWindow("Trackbars", 0)

        for k,j in enumerate("HSV"):
            cv2.createTrackbar("min_%s" % (j), "Trackbars", self.min_values[k], 255, self.tracker_callback)
        for k,j in enumerate("HSV"):
            cv2.createTrackbar("max_%s" % (j), "Trackbars", self.max_values[k], 255, self.tracker_callback)

def main():
    print("please, write the color that you want to calibrate or the name of new one")

    color_name = raw_input("color to calibrate (Enter): ")
    c = color_detection(color=color_name)
    img_topic = "/usb_cam/image_raw"#"/bebop/image_raw"#
    bridge = CvBridge()
    rospy.init_node('test_ros_image', anonymous=True)
    
    while not rospy.is_shutdown():
        data = rospy.wait_for_message(img_topic, Image)
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if not (cols > 60 and rows > 60):  # returns if data have unvalid shape
            continue
        c.get_rect(cv_image)
        k = cv2.waitKey(5)
        if k == 27 : break  #esc pressed
        elif  k == ord("s"): c.save_color()
 
if __name__ == '__main__':
    main()