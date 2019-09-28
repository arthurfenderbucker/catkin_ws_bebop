#!/usr/bin/env python
import rospy
#from __future__ import print_function
import cv2
import numpy as np
import rospkg
import json
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int16

from geometry_msgs.msg import Point


from cv_bridge import CvBridge, CvBridgeError


#rospack = rospkg.RosPack()

#config_path = str(rospack.get_path('cv_detection')+'/config/feature_config.json')
#images_path = str(rospack.get_path('cv_detection')+'/config/img/')
bridge = CvBridge()

class feature_detection():
    def __init__(self,image_name="H.png", image_topic="/usb_cam/image_raw", pub_topic="/cv_detection/feature_detection/detection", reference = "x"):

        rospy.init_node('feature_detection', anonymous = True)


        #with open(config_path) as json_data_file:
        #    self.config_data = json.load(json_data_file)


        self.reference = rospy.get_param('~reference', reference)
        self.img_topic = rospy.get_param('~image_topic', image_topic)
        self.pub_topic = rospy.get_param('~pub_topic', pub_topic)
        self.running = rospy.get_param('~running', True)


        self.image_topic_sub = rospy.Subscriber(
            "cv_detection/feature_detection/set_img_topic", String, self.set_image_topic, queue_size=None)
        self.pub_topic_sub = rospy.Subscriber(
            "cv_detection/feature_detection/set_pub_topic", String, self.set_pub_topic, queue_size=None)
        self.running_sub= rospy.Subscriber(
            "cv_detection/feature_detection/set_runnig_state", Bool, self.set_runninng_state, queue_size=None)

        self.reference_topic_sub = rospy.Subscriber(
            "cv_detection/feature_detection/set_reference_topic", String, self.set_reference_topic, queue_size = None)

        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)



        #load the image
        self.source_image = cv2.imread( self.img_topic,cv2.IMREAD_GRAYSCALE )
        #print(self.source_image.shape)


        # Initiate ORB detector
        self.orb = cv2.ORB_create()

        self.ref_key_points, self.ref_description =  self.orb.detectAndCompute(self.source_image,None)


        # FLANN parameters
        # FLANN_INDEX_KDTREE = 1
        # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        # search_params = dict(checks=5)   # or pass empty dictionary
        # # create BFMatcher object
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # self.flann = cv2.FlannBasedMatcher(index_params,search_params)
        #configure plt
        plt.ion()
        plt.show()


#============= callbacks =======================

    def set_image_topic(self, topic_name):
        self.img_topic = topic_name.data


    def set_pub_topic(self, topic_name):
        self.pub_topic = topic_name.data
        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)

    def set_runninng_state(self,boolean_state):
        self.running = boolean_state.data
        if not self.running:
            cv2.destroyAllWindows()

    def set_reference_topic(self, topic_name):
        self.reference = reference.data

#===============================================================


    def get_rect(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        print(image.shape)
        key_points, description =  self.orb.detectAndCompute(image,None)

        #calculate matches between 2 descriptions
        matches = self.bf.match(self.ref_description,description)
        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)


        img3 = cv2.drawMatches(self.source_image,self.ref_key_points,image,key_points,matches[:10],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        plt.imshow(img3,)
        plt.draw()
        plt.pause(0.001)

    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                data = rospy.wait_for_message(self.img_topic, Image)
                self.source_image = cv2.imread( self.image_topic,cv2.IMREAD_GRAYSCALE )
                self.ref_key_points, self.ref_description =  self.orb.detectAndCompute(self.source_image,None)

                try:
                    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                except CvBridge as e:
                    print(e)

                (rows, cols, channels) = cv_image.shape
                if not (cols > 60 and rows > 60):
                    continue
                center,radius,rect = self.get_rect(cv_image)

                if center != None:

                     img_crop, img_rot = self.crop_rect(cv_image,rect)
                     p = Point()
                     p.x = center[0]
                     p.y = center[1]
                     p.z = radius[0][0][0]
                     print(radius[0][0][0])
                     if radius > self.min_radius:
                         self.ref_pub.publish(p)

            k = cv2.waitKey(5)
            if k == 27 : break  #esc pressed
            elif rospy.get_param('~calibrate',False) and k == ord("s") : self.save_color()




def main():

    print("init")
    f = feature_detection()
    f.run()

if __name__ == '__main__':
    main()
