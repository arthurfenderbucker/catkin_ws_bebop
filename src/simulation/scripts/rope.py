#!/usr/bin/env python
# optical flow
from __future__ import print_function

# import mavros
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import argparse
import time
from dronecontrol.msg import Vector3D
from optical_flow import CameraStabilization as camStab
from line_follow import line_follow


class Rope:

    image_sub = None

    def __init__(self):

        rospy.Subscriber("/state_machine/state",
                         String, self.state_callback)
        self.pub_condition = rospy.Publisher(
            "/state_machine/rope/find_condition", String, queue_size=10)
        rospy.init_node('rope', anonymous=True)

    def setup_wire(self):

        self.vel_pub = rospy.Publisher(
            'controle/velocity', Vector3D, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera_down/image_raw", Image, self.callback)
        # self.stab = camStab()
        self.line_f = line_follow()
        self.vec_vel = Vector3D()
        self.vec_vel.x = 0
        self.vec_vel.y = 0
        self.vec_vel.z = 0

        self.step = 0

    def state_callback(self, data):
        if (data.data == "find_rope"):
            rospy.loginfo(" WIRE !!!")
            self.setup_wire()
            # condition!!
        elif(self.image_sub != None):
            self.image_sub.unregister()
            self.image_sub = None
            cv2.destroyAllWindows()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if not (cols > 60 and rows > 60):  # returns if data have unvalid shape
            return
        self.line_f.update(cv_image)

        k = cv2.waitKey(30) & 0xff
        if k == 27:
            exit()
            # self.stab.resetFeatures(cv_image)

        try:
            # self.vel_pub.publish(self.vec_vel)
            # print(self.vec_vel.x)
            #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            pass
        except CvBridgeError as e:
            print(e)

    # def detectCorners(self, img):

    #     features_drift, global_drift = self.stab.getDrift()
    #     features_pos, global_pos = self.stab.getPosition()

    #     cv2.imshow("Original topic window", img)
    #     try:
    #         self.total_feature_drift += features_drift
    #     except:
    #         self.total_feature_drift = features_drift

    #     print(global_drift.mean())


def main():

    print("init")

    ic = Rope()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
