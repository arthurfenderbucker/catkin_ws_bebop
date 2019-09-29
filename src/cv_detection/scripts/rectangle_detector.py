#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import time
import cv_common

from cv_bridge import CvBridge, CvBridgeError

import rospkg
rospack = rospkg.RosPack()
detections_path = str(rospack.get_path('cv_detection')+'/imgs/rectangle/')


bridge = CvBridge()

class rectangle_detector(object):
    """docstring for detect_rectangle"""


    def __init__(self,image_topic="/usb_cam/image_raw", pub_topic="/cv_detection/rectangle_detector/detection"):
        super(rectangle_detector, self).__init__()
        self.colors = np.random.randint(0, 255, (30, 3))
        rospy.init_node('rectangle_detector', anonymous=True)
        print("init")
        self.img_topic = rospy.get_param('~image_topic',image_topic)
        self.pub_topic = rospy.get_param('~pub_topic',pub_topic)
        # self.crop_rect = rospy.get_param('~crop_rect',False)
        # self.cropped_rect_pub_topic = rospy.get_param('~cropped_rect_pub_topic','/cv_detection/rectangle_detector/cropped_rect')
        # self.save_detection = rospy.get_param('~save_detection',False)
        self.save_detection_path = rospy.get_param('~save_detection_path',detections_path)
        self.save_detection_name = rospy.get_param('~save_detection_path','default.png')
        self.filter_rotation = rospy.get_param('~filter_rotation',False)

        self.running = rospy.get_param('~running',True)

        self.image_topic_sub = rospy.Subscriber(
            "cv_detection/rectangle_detector/set_img_topic", String, self.set_image_topic, queue_size=1)
        self.pub_topic_sub = rospy.Subscriber(
            "cv_detection/rectangle_detector/set_pub_topic", String, self.set_pub_topic, queue_size=1)
        self.running_sub= rospy.Subscriber(
            "cv_detection/rectangle_detector/set_running_state", Bool, self.set_running_state, queue_size=1)
        
        self.save_detection_sub = rospy.Subscriber(
            "cv_detection/rectangle_detector/save_detection", String, self.save_detection, queue_size=1)
        
        self.save_image_raw_sub = rospy.Subscriber(
            "cv_detection/rectangle_detector/save_image_raw", String, self.save_image_raw, queue_size=1)
        
        self.filter_rotation_sub = rospy.Subscriber(
            "cv_detection/rectangle_detector/set_filter_rotation", Bool, self.set_filter_rotation, queue_size=1)

        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)
        self.detection_saved_pub = rospy.Publisher('cv_detection/rectangle_detector/detection_saved',Bool, queue_size=1)

        self.count_fps = 0
        self.t_0 = 0
        self.rect = None
    def set_image_topic(self, topic_name):
        self.img_topic = topic_name.data

    def set_pub_topic(self, topic_name):
        self.pub_topic = topic_name.data
        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)

    def set_running_state(self,boolean_state):
        self.running = boolean_state.data
        if not self.running:
            cv2.destroyAllWindows()
    def set_filter_rotation(self,data):#Bool
        self.filter_rotation = data.data

    def save_image_raw(self,file_name):
        if not self.image is None:
            cv2.imwrite(self.save_detection_path+'frame/'+file_name.data, self.image[100:-100,100:-100])                
        else:
            rospy.logerr("no image received  yet")
    def save_detection(self,file_name):
        
        if not self.rect == None:
            img_crop, img_rot = self.crop_rect(self.image,self.rect)
            try:
                print("crop saved to:")
                print(self.save_detection_path+'cropped/'+file_name.data)
                cv2.imwrite(self.save_detection_path+'cropped/'+file_name.data, img_crop)
                cv2.imwrite(self.save_detection_path+'frame/'+file_name.data, self.detected_image)

                self.detection_saved_pub.publish(True)
            except:
                self.detection_saved_pub.publish(False)
        else:
            print("no rects detecteed")
            self.detection_saved_pub.publish(False)
                

    def crop_rect(self, img, rect):
        # get the parameter of the small rectangle
        center, size, angle = rect[0], rect[1], rect[2]
        center, size = tuple(map(int, center)), tuple(map(int, size))

        # get row and col num in img
        height, width = img.shape[0], img.shape[1]

        # calculate the rotation matrix
        M = cv2.getRotationMatrix2D(center, angle, 1)
        # rotate the original image
        img_rot = cv2.warpAffine(img, M, (width, height))

        # now rotated rectangle becomes vertical and we crop it
        img_crop = cv2.getRectSubPix(img_rot, size, center)

        return img_crop, img_rot

    def update(self, image,show=True):

        # convert the image to grayscale, blur it, and find edges
        # in the image
        img = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #------------- contours, area & perpendicular approach -----------------
        for i in range(5):
            gray = cv2.bilateralFilter(gray, 7, 3, 11)
        cv2.imshow("gray bi", gray)
        edged = cv2.Canny(gray, 30, 200, apertureSize=5)
        cv2.imshow("edged", edged)

        # find contours in the edged image, keep only the ones with the higher area
        im2, cnts, _ = cv2.findContours(edged.copy(),
                                        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:30]
        poligons = cv_common.filter_cnts(cnts, filter_rotation=True, max_sides=4)
        
        if len(poligons) >0:
            # print("---")
            # print(poligons[0])
            ((x,y), radius) = cv2.minEnclosingCircle(poligons[0])
            rect = cv2.minAreaRect(poligons[0])

            if show and radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(image, (int(x), int(y)), 3, (0, 0, 255), -1)
                cv2.putText(image,"centroid", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
                # cv2.putText(image,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)

            if show: 

                cv_common.show_poligons(image, poligons)
            
            return (x,y),radius, rect
        else:
            return None, None, None
            

    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                data = rospy.wait_for_message(self.img_topic, Image)
                try:
                    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                except CvBridgeError as e:
                    print(e)

                (rows, cols, channels) = cv_image.shape
                if not (cols > 60 and rows > 60):  # returns if data have unvalid shape
                    continue
                
                self.image = cv_image.copy()
                center, radius, rect = self.update(cv_image)
                self.detected_image = cv_image
                
                # print("fps: " +str(1/(time.time() - self.t_0)))
                # self.t_0 = time.time()
                # print("radius: "+str(radius))
                if not center is None:
                    self.rect = rect
                    p = Point()
                    (p.x, p.y) = center
                    p.z = radius
                    # print(p)
                    if radius > 30: 
                        self.ref_pub.publish(p)  
            k = cv2.waitKey(1)
            # if k == 27 : break  #esc pressed


if __name__ == "__main__":

    d = rectangle_detector()
    d.run()

    cv2.destroyAllWindows()
    # cap.release()

