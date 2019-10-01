#!/usr/bin/env python
import rospy
#from __future__ import print_function
import cv2
import numpy as np
import json
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int16
import cv_common
from geometry_msgs.msg import Point


from cv_bridge import CvBridge, CvBridgeError


import rospkg
rospack = rospkg.RosPack()

# config_path = str(rospack.get_path('cv_detection')+'/config/feature_config.json')
images_path = str(rospack.get_path('cv_detection')+'/imgs/rectangle/cropped/')
bridge = CvBridge()

class feature_detector():
    def __init__(self,image_name="test.png", image_topic="/usb_cam/image_raw", pub_topic="/cv_detection/feature_detector/detection"):

        rospy.init_node('feature_detector', anonymous = True)


        #with open(config_path) as json_data_file:
        #    self.config_data = json.load(json_data_file)


        image_name = rospy.get_param('~ref_image', image_name)
        self.img_topic = rospy.get_param('~image_topic', image_topic)
        self.pub_topic = rospy.get_param('~pub_topic', pub_topic)
        self.running = rospy.get_param('~running', True)


        self.image_topic_sub = rospy.Subscriber(
            "cv_detection/feature_detector/set_img_topic", String, self.set_image_topic, queue_size=1)
        self.pub_topic_sub = rospy.Subscriber(
            "cv_detection/feature_detector/set_pub_topic", String, self.set_pub_topic, queue_size=1)
        self.running_sub= rospy.Subscriber(
            "cv_detection/feature_detector/set_running_state", Bool, self.set_running_state, queue_size=1)
        self.ref_image_sub = rospy.Subscriber(
            "cv_detection/feature_detector/set_ref_image", String, self.set_ref_image, queue_size = 1)

        
            

        self.raw_features_pub = rospy.Publisher("/cv_detection/feature_detector/features_center", Point, queue_size=1)
        
        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)
        
        self.source_image = cv2.imread(images_path+image_name,cv2.IMREAD_GRAYSCALE )

        # Initiate ORB detector
        self.orb = cv2.ORB_create()

        self.ref_key_points, self.ref_description =  self.orb.detectAndCompute(self.source_image,None)

        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)



#============= callbacks =======================

    def set_image_topic(self, topic_name):
        self.img_topic = topic_name.data


    def set_pub_topic(self, topic_name):
        self.pub_topic = topic_name.data
        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)

    def set_running_state(self,boolean_state):
        self.running = boolean_state.data
        if not self.running:
            cv2.destroyAllWindows()

    def set_ref_image(self, image_name):
        self.source_image = cv2.imread(str(images_path+image_name.data),cv2.IMREAD_GRAYSCALE )
        self.ref_key_points, self.ref_description =  self.orb.detectAndCompute(self.source_image,None)
        print(image_name)


#===============================================================


    def get_rect(self, image, show = True, draw_matches = True):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        key_points, description =  self.orb.detectAndCompute(image,None)

        #calculate matches between 2 descriptions
        try:
            matches = self.bf.match(self.ref_description,description)
        except:
            return None,None,None, None
        #  print(matches)
        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)
        good_matches = matches[:30]
        # good_matches = filter(lambda x:x.distance < 60.0,matches)
        if len(good_matches)>5:
            src_pts = np.float32([  self.ref_key_points[m.queryIdx].pt for m in good_matches     ]).reshape(-1,1,2)
            dst_pts = np.float32([ key_points[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = self.source_image.shape[:2]
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            print( np.mean(dst_pts, axis=0))
            if draw_matches:
                dst_show = dst + (w, 0)  # adding offset
                draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask, # draw only inliers
                            flags = 2)

                img3 = cv2.drawMatches(self.source_image, self.ref_key_points,image,key_points,good_matches, None,**draw_params)

                # Draw bounding box in Red
                img3 = cv2.polylines(img3, [np.int32(dst_show)], True, (0,0,255),3, cv2.LINE_AA)

                cv2.imshow("result", img3)

            poligons = cv_common.filter_cnts([np.squeeze(dst).astype(int)],filter_rotation=False, max_sides = 4,max_perpendicular_factor = 30)

            if len(poligons)>0:
                ((x,y), radius) = cv2.minEnclosingCircle(poligons[0])
                rect = cv2.minAreaRect(poligons[0])

                if show and radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    cv2.circle(image, (int(x), int(y)), 3, (0, 0, 255), -1)
                    cv2.putText(image,"centroid", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
                if show: 
                    cv_common.show_poligons(image, poligons)

                return (x,y),radius, rect, np.mean(dst_pts, axis=0)[0]
            
        return None, None, None, None

    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                data = rospy.wait_for_message(self.img_topic, Image)

                try:
                    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                except CvBridge as e:
                    print(e)

                (rows, cols, channels) = cv_image.shape
                if not (cols > 60 and rows > 60):
                    continue
                center,radius,rect, features_center = self.get_rect(cv_image)
                print(radius)
                if not features_center is None:
                    p_features = Point()
                    p_features.x,p_features.y = features_center.tolist()
                    self.raw_features_pub.publish(p_features)

                if not center is None:
                    self.rect = rect
                    p = Point()
                    (p.x, p.y) = center
                    p.z = radius
                    # print(p)
                    if radius > 30: 
                        self.ref_pub.publish(p)  

            k = cv2.waitKey(5)
            if k == 27 : break  #esc pressed




def main():

    
    f = feature_detector()
    f.run()

if __name__ == '__main__':
    main()
