#!/usr/bin/env python

# import mavros
import sys
import rospy

import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from dronecontrol.msg import Vector3D

bridge = CvBridge()
cap = cv2.VideoCapture(0)

def callback( data):
    global bridge
    print("callback")
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    # ret, cv_image = self.cap.read()

    cv_image = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)
    (rows, cols, channels) = cv_image.shape
    print(cv_image.shape)
    if not (cols > 60 and rows > 60):  # returns if data have unvalid shape
        return
    _,f = cap.read()
    cv2.imshow("ground", f)


def main():

    print("init")
    rospy.init_node('window', anonymous=True)
    image_sub = rospy.Subscriber("/bebop/image_raw", Image, callback, queue_size=1)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
