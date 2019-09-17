#!/usr/bin/env python
# optical flow

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
from optical_flow import CameraStabilization as camStab

import PIL


class DroneStabilization:

    image_sub = None

    def __init__(self):

        rospy.Subscriber("/state_machine/state",
                         String, self.state_callback)  #initialize the callback to listten to SM state

        self.pub_condition = rospy.Publisher(
            "/state_machine/stabilize/find_condition", String, queue_size=10) #publisher to notify when the state is done

        self.pub_condition = rospy.Publisher(
            "/control/stabilize/offset", Vector3D, queue_size=1) #publisher to notify when the state is done

        rospy.init_node('stabilize', anonymous=True)

        self.count_stable = 0

    def setup_stabilization(self):

        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
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
            "/bebop/image_raw", Image, self.callback, queue_size=None)

        rospy.loginfo("setup ok")
        # self.pub_condition.publish("ok")

    def state_callback(self, data):
        if (data.data == "stabilize"):
            rospy.loginfo(" stabilize !!!")
            self.setup_stabilization()
            # condition!!
        elif(self.image_sub != None):
            rospy.loginfo(" end! ")
            cv2.destroyAllWindows()
            self.image_sub.unregister() #unsubscribe from image topics
            self.image_sub = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        if not (cols > 60 and rows > 60):  # returns if data have unvalid shape
            return
        # self.detect.update(cv_image)

        if self.frame == 0:
            self.opt_flow.resetFeatures(cv_image, get_corners_only=False)
            self.frame += 1
        else:
            self.opt_flow.update(cv_image)
            print(self.opt_flow.get_square_shape())
            self.adjust_position(
                self.opt_flow.get_square_center(),self.opt_flow.last_reset_center, self.opt_flow.get_square_shape())

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

    def adjust_position(self, window_center_pos,ref_center, window_shape, ideal_window_image_rate=0.60):
        delta = ref_center - window_center_pos
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

    ic = DroneStabilization()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


# else:
#     self.mean_box_cord = self.mean_box_cord * \
#         0.8 + m_box[:] * 0.2  # impulse filter

 # mean_box_center = np.array([(self.mean_box_cord[3] + self.mean_box_cord[1]) / 2,
#                             (self.mean_box_cord[2] + self.mean_box_cord[0]) / 2])
# img = cv2.circle(
#     img, (self.mean_box_cord[1], self.mean_box_cord[0]), 5, (200, 0, 0), -1)
# img = cv2.circle(
#     img, (self.mean_box_cord[3], self.mean_box_cord[2]), 5, (200, 0, 0), -1)
# img = cv2.circle(img, (int(mean_box_center[0]), int(
#     mean_box_center[1])), 5, (0, 200, 0), -1)
