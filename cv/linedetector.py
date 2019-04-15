#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class LineDetector:
    def __init__(self):
        self.frame = 0
        self.bridge = CvBridge()
        self.gray = np.zeros((256, 256, 1), dtype = "uint8")
        self.cv_image =  np.zeros((256, 256, 1), dtype = "uint8")
        self.edges = np.zeros((256, 256, 1), dtype = "uint8")
        self.lines =  np.zeros((256, 256, 1), dtype = "uint8")
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback, queue_size=1)

    def image_callback(self, image):
        if self.frame % 3 == 0:
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(image, "mono8")
                self.gray = self.cv_image
                cv2.imshow("Camera", self.gray)
                self.lines, self.edges = self.detectLines()
                rospy.spin()
            except CvBridgeError as e:
                print ("CvBridgeError: " + str(e))
            cv2.waitKey(3)
        self.frame += 1

    def detectLines():
        edges = cv2.Canny(self.gray,50,150,apertureSize = 3)
        self.lines_img = img
        self.edges_img = img
        for edge in edges:
            cv2.circle(edges_img, (edge[0], edge[1]), 5,(0,255,0),10)
        # This returns an array of r and theta values
        lines = cv2.HoughLines(edges,1,np.pi/180, 200)
        # The below for loop runs till r and theta values
        # are in the range of the 2d array
        for r,theta in lines[0]:
            #if abs(theta) <
            # Stores the value of cos(theta) in a
            a = np.cos(theta)
            # Stores the value of sin(theta) in b
            b = np.sin(theta)
            # x0 stores the value rcos(theta)
            x0 = a*r
            # y0 stores the value rsin(theta)
            y0 = b*r
            # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
            x1 = int(x0 + 1000*(-b))
            # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
            y1 = int(y0 + 1000*(a))
            # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
            x2 = int(x0 - 1000*(-b))
            # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
            y2 = int(y0 - 1000*(a))
            # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
            # (0,0,255) denotes the colour of the line to be
            #drawn. In this case, it is red.
            cv2.line(lines_img,(x1,y1), (x2,y2), (0,0,255),2)
            print(theta)
        cv2.imshow('lines', lines)
        return lines_img, edges_img

rospy.init_node('LineDetector')
rate = rospy.Rate(20)
lindec = LineDetector()
while(True):

    cv2.imshow('edges', lindec.edges)
    cv2.imshow('lines', lindec.lines)
    cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
