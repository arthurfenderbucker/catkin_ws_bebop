#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('WebcamVideoPublisher', anonymous=True)

VideoRaw = rospy.Publisher('camera/image_raw', Image, queue_size=10)
cap = cv2.VideoCapture(0)

print("start")
while cap.isOpened():
    ret, frame = cap.read()
    cv2.imshow("Image window", frame)
    k = cv2.waitKey(1)
    if k == 27: #esc:
        break
    try:
        msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
        VideoRaw.publish(msg_frame)
    except CvBridgeError as e:
        print(e)
print("end")
cv2.destroyAllWindows()
    
