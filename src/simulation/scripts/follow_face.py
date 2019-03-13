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
from detect_window import detect_window

# from yolo3.yolo import YOLO, detect_video
import PIL



class DroneStabilization:

    image_sub = None

    def __init__(self):
        print(cv2. __version__)
        rospy.Subscriber("/state_machine/state",
                         String, self.state_callback)
        self.pub_condition = rospy.Publisher(
            "/state_machine/follow/find_condition", String, queue_size=10)
        rospy.init_node('follow_faces', anonymous=True)
        # self.yolo = YOLO()

        self.count_stable = 0

        self.face_cascade = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml')
        if self.face_cascade.empty(): print("your face_cascade is empty. are you sure, the path is correct ?")


        self.center = np.array([100,100,0,0])
        self.alpha = 0.1
        # self.setup_window()

    def setup_window(self):

        # self.image_pub = rospy.Publisher("image2", Image, queue_size=10)
        self.vel_pub = rospy.Publisher(
            'controle/velocity', Vector3D, queue_size=10)

        self.bridge = CvBridge()

        # self.stab = camStab()
        self.detect = detect_window()
        self.vec_vel = Vector3D()
        self.vec_vel.x = 0
        self.vec_vel.y = 0
        self.vec_vel.z = 0
        self.pixel_movement = [0, 0]

        self.frame = 0
        self.count_callbacks = 0
        self.opt_flow = camStab()
        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw", Image, self.callback, queue_size=1)

        rospy.loginfo("setup ok")
        # self.pub_condition.publish("ok")

    def state_callback(self, data):
        if (data.data == "follow"):
            rospy.loginfo(" FOLLOW !!!")
            self.setup_window()
            # condition!!
        elif(self.image_sub != None):
            rospy.loginfo(" end! ")
            cv2.destroyAllWindows()
            self.image_sub.unregister()
            self.image_sub = None

    def callback(self, data):
        print("ksadfnk")
        # if self.count_callbacks < 10:
        #     self.count_callbacks += 1
        #     return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if not (cols > 60 and rows > 60):  # returns if data have unvalid shape
            return
        # self.detect.update(cv_image)
        cv2.imshow("frame", cv_image)
        cv_image = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        if len(faces)>0:
            self.center= self.center*(1-self.alpha)+np.mean(faces,axis=0)*self.alpha
            print(self.center)
            for (x,y,w,h) in faces:
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.circle(cv_image,(int(self.center[0]),int(self.center[1])),10,(0,0,200),-1)
            # roi_gray = gray[y:y+h, x:x+w]
            # roi_color = cv_image[y:y+h, x:x+w]
            # eyes = eye_cascade.detectMultiScale(roi_gray)
            # for (ex,ey,ew,eh) in eyes:
            #     cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        cv2.imshow('Video', cv_image)

        k = cv2.waitKey(30) & 0xff
        if k == 27:
            exit()
        elif k == 'r' or k == ord('r'):
            print("r pressed ")
            self.frame = 0
        elif k == ord('p') or k == 'p':
            print("through_window ")
            self.pub_condition.publish("ok")

        try:
            # self.vel_pub.publish(self.vec_vel)
            # print(self.vec_vel.x)
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            pass
        except CvBridgeError as e:
            print(e)
        self.count_callbacks = 0

    def adjust_position(self, window_center_pos, window_shape, ideal_window_image_rate=0.60):
        delta = self.image_center - window_center_pos
        # rospy.loginfo(delta)
        self.vec_vel.x = 0
        self.vec_vel.y = 0
        self.vec_vel.z = 0
        if abs(delta[0]) > 5 or abs(delta[1]) > 5:
            self.vec_vel.y = np.sign(
                delta[0]) * min(np.abs(delta[0]) / 20, 0.2)
            self.vec_vel.z = np.sign(
                delta[1]) * min(np.abs(delta[1]) / 20, 0.2)
            self.count_stable = 0

        rate = window_shape[1] / self.image_h
        delta_rate = ideal_window_image_rate - rate
        # (x,y) goes back
        if abs(delta_rate) > 0.02 and abs(delta[0]) < 20 and abs(delta[1]) < 20:
            self.vec_vel.x = delta_rate * 2
            self.count_stable = 0

        self.vel_pub.publish(self.vec_vel)
        self.count_stable += 1
        if self.count_stable > 3:
            rospy.loginfo("GOOD!!")
            self.pub_condition.publish("ok")


def main():

    print("init")

    ic = DroneStabilization()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
