#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from color_detection import color_detection
from std_msgs.msg import Bool
# from feature_detection import feature_detection

img_topic = "/bebop/image_raw"#"/usb_cam/image_raw"#
bridge = CvBridge()

class pickup_box:
    def __init__(self,color="red"):
        # self.sub_pos = rospy.Subscriber(
        #     "mavros/local_potion/pose", PoseStamped, self.callback)
        rospy.init_node('pickup_box', anonymous=True)


        self.aligned_sub = rospy.Subscriber("/control/align_reference/aligned", Bool, take_box, queue_size=1)
        self.image_sub = rospy.Subscriber( img_topic, Image, self.img_callback, queue_size=None)

        self.ref_pub = rospy.Publisher('/control/align_reference/ref_point', Point, queue_size=1)

        self.c = color_detection(color)



        self.pub_condition = rospy.Publisher(
            "/state_machine/takeoff_condition", String, queue_size=1)

        self.pub_position = rospy.Publisher(
            '/controle/position', Vector3D, queue_size=10)

        # rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
        #                  Ardrone3PilotingStateAltitudeChanged, self.callback)

        self.counter = 0
        self.z = 0
        self.pos = Vector3D()
        self.pos.x = 0
        self.pos.y = 0
        self.pos.z = 1.3

    def callback(self, data):
        return
    def state_callback(self, data):
        print("got ")
        print(data.data)
        rospy.loginfo(type(str(data)))
        if (data.data == "takeoff"):

            self.sub_pos = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                                            Ardrone3PilotingStateAltitudeChanged, self.callback)
            self.pub_position.publish(self.pos)

            if self.z > 0.7:
                self.pub_condition.publish("ok")
        elif(self.sub_pos != 0):
            self.sub_pos.unregister()

            # print("exited")

    def execute(self, userdata):
        rospy.loginfo('Executing state takeoff')
        if self.counter < 3:
            self.counter += 1
            return 'flying'
        else:
            return 'erro'


def main():

    print("init")
    t = takeoff()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
