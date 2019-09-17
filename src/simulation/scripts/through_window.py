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
from drone_control.msg import Vector3D


class through_window(object):
    """docstring for through_window"""

    image_sub = None

    def __init__(self):
        super(through_window, self).__init__()

        rospy.Subscriber("/state_machine/state",
                         String, self.state_callback)
        self.pub_condition = rospy.Publisher(
            "/state_machine/window/through_condition", String, queue_size=10)
        rospy.init_node('window', anonymous=True)

        self.vel_pub = rospy.Publisher(
            'controle/velocity', Vector3D, queue_size=10)

        self.vec_vel = Vector3D()

    def setup_through_window(self):
        self.image_sub = rospy.Subscriber(
            "/camera_down/image_raw", Image, self.callback, queue_size=None)

        self.bridge = CvBridge()

        self.vec_vel.x = 2
        self.vec_vel.y = 0
        self.vec_vel.z = 0
        self.vel_pub.publish(self.vec_vel)
        self.init_time = time.time()

        # time.sleep(3)
        # self.vec_vel.x = 0
        # self.vec_vel.y = 0
        # self.vec_vel.z = 0
        # self.vel_pub.publish(self.vec_vel)

        # self.pub_condition.publish("ok")

        rospy.loginfo("setup ok")

    def state_callback(self, data):
        if (data.data == "through_window"):
            rospy.loginfo(" THROUGH WINDOW !!!")
            self.setup_through_window()
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

        cv2.imshow("ground", cv_image)
        if time.time() - self.init_time > 4:
            self.vec_vel.x = 0
            self.vec_vel.y = 0
            self.vec_vel.z = 0
            self.vel_pub.publish(self.vec_vel)
            self.pub_condition.publish("ok")

        k = cv2.waitKey(30) & 0xff
        if k == 27:
            exit()
        elif k == 'r' or k == ord('r'):
            print("r pressed ")
            self.frame = 0
        elif k == ord('p') or k == 'p':
            print("through_window ")
            # self.pub_condition.publish("ok")


def main():

    print("init")

    tw = through_window()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
