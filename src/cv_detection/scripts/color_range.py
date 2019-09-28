#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int16

from geometry_msgs.msg import Point
import rospkg

from cv_bridge import CvBridge, CvBridgeError

import json

rospack = rospkg.RosPack()
config_path = str(rospack.get_path('cv_detection')+'/config/color_config.json')
bridge = CvBridge()

class color_range():
    colors = np.random.randint(0, 255, (10, 3))
    color_tracking = True
    
    
    min_values = [0,0,0]
    max_values = [255,255,255]

    def __init__(self,color="green", image_topic="/usb_cam/image_raw", pub_topic="/cv_detection/color_range/detection"):
        #get min and max HSV values from config file



        rospy.init_node('color_range', anonymous=True)

        with open(config_path) as json_data_file:
            self.config_data = json.load(json_data_file)

        self.color = rospy.get_param('~color',color)
        self.img_topic = rospy.get_param('~image_topic',image_topic)
        self.pub_topic = rospy.get_param('~pub_topic',pub_topic)
        self.calibrating = rospy.get_param('~calibrate',False)
        self.running = rospy.get_param('~running',True)
        self.min_radius = rospy.get_param('~min_radius',20)

        print("saved colors: ")
        for k in  self.config_data.keys():
            print(k + " : min "+ str(self.config_data[k]["min"].values()) + " max" +str(self.config_data[k]["max"].values() ) )

        if self.color in self.config_data.keys():
            print("existing color selected!")     
        else:
            print("new color selected!")
            self.config_data[self.color] = {"min":{"H":0,"S":0,"V":0}, "max":{"H":255,"S":255,"V":255}}

        self.min_values = self.config_data[color]["min"].values()
        self.max_values = self.config_data[color]["max"].values()
        
        if self.calibrating: self.calibrate()
        
        print(self.color)
               
        self.image_topic_sub = rospy.Subscriber(
            "cv_detection/color_range/set_img_topic", String, self.set_image_topic, queue_size=None)
        self.pub_topic_sub = rospy.Subscriber(
            "cv_detection/color_range/set_pub_topic", String, self.set_pub_topic, queue_size=None)
        self.running_sub= rospy.Subscriber(
            "cv_detection/color_range/set_running_state", Bool, self.set_running_state, queue_size=None)
        self.color_sub = rospy.Subscriber(
            "cv_detection/color_range/set_color", String, self.set_color, queue_size=None)
        self.min_radius_sub = rospy.Subscriber(
            "cv_detection/color_range/set_min_radius", Int16, self.set_min_radius, queue_size=None)

        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)
        self.crop_pub = rospy.Publisher("cv_detection/color_range/detection", Image, queue_size=1)
    
    # ====================== callbacks ===========================
    def set_image_topic(self, topic_name):
        self.img_topic = topic_name.data

    def set_pub_topic(self, topic_name):
        self.pub_topic = topic_name.data
        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)

    def set_running_state(self,boolean_state):
        self.running = boolean_state.data
        if not self.running:
            cv2.destroyAllWindows()
 
    def set_min_radius(self, data):
        self.min_radius = data.data

    def set_color(self,color):

        if not color.data in self.config_data.keys():
            rospy.loginfo("""ATENTION!!! COLOR DOES NO EXIST ON THE DATASET, THE COLOR WAS NOT UPDATED!
            Please clibrate this color using the launch file: calibrate_color_range.launch """)
        else:
            self.color = color.data
            self.update_color()
            if self.calibrating: #update trackbars
                self.update_trackbar()
            rospy.loginfo("Color updated to: "+color.data)
    
    # =================================================================================

    def update_color(self):
        for k,j in enumerate("HSV"):

            if self.color in self.config_data.keys():
                self.min_values[k] = self.config_data[self.color]["min"][j]
                self.max_values[k] = self.config_data[self.color]["max"][j]

    def update_trackbar(self):
        for k,j in enumerate("HSV"):
            cv2.setTrackbarPos("min_%s" % (j), "Trackbars",self.min_values[k])
            cv2.setTrackbarPos("max_%s" % (j), "Trackbars",self.max_values[k])


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
    def nothing(self,x):
        pass
    def get_rect(self,image, show = True):
        
        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert to HSV
        # cv2.imshow("Trackbars",self.color_range_image)
        if self.min_values[0]<self.max_values[0]: #for colors close to red with hue value close to 0 and 180 
            thresh = cv2.inRange(frame_to_thresh,tuple(self.min_values), tuple(self.max_values))
        else:
            thresh1 = cv2.inRange(frame_to_thresh,(0,self.min_values[1],self.min_values[2]), (self.max_values[0],self.max_values[1],self.max_values[2]))
            thresh2 = cv2.inRange(frame_to_thresh,(self.min_values[0],self.min_values[1],self.min_values[2]), (180,self.max_values[1],self.max_values[2]))
            
            thresh = cv2.bitwise_or(thresh1, thresh2)

        # ignore random noise and merge very close areas
        kernel = np.ones((5,5),np.uint8) 
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # print(cv2.mean(image,mask))
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            rect = cv2.minAreaRect(c)
            img_crop, img_rot = self.crop_rect(image, rect)
            
            # print(rect)  
            # if show:
            #     cv2.imshow("rect",img_crop)
            # print(rect)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if show:
                cv2.drawContours(image,[box],0,(0,0,255),2)

            # only proceed if the radius meets a minimum size
            if show and radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(image, center, 3, (0, 0, 255), -1)
                cv2.putText(image,"centroid", (center[0]+10,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
                cv2.putText(image,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)

            
            # show the frame to our screen
        
                if show :
                    cv2.imshow("Original", image)
                    cv2.imshow("Thresh", thresh)
                    # cv2.imshow("Mask", mask)
            return [center,radius,rect]
        # if show :
        #     cv2.imshow("Original", image)
        #     cv2.imshow("Thresh", thresh)
            # cv2.imshow("Mask", mask)
        return [None, None, None]

    def save_color(self):
        with open(config_path, 'w') as json_data_file:
            json.dump(self.config_data, json_data_file)
        print("changes saved!")


    def tracker_callback(self,x):
        for k,j in enumerate("HSV"):
            self.min_values[k] = cv2.getTrackbarPos("min_%s" % (j), "Trackbars")
            self.config_data[self.color]["min"][j] = self.min_values[k]

            self.max_values[k] = cv2.getTrackbarPos("max_%s" % (j), "Trackbars")
            self.config_data[self.color]["max"][j] = self.max_values[k]
        

    def setup_trackbars(self):
        cv2.namedWindow("Trackbars", 0)

        for k,j in enumerate("HSV"):
            cv2.createTrackbar("min_%s" % (j), "Trackbars", self.min_values[k], 255, self.tracker_callback)
        for k,j in enumerate("HSV"):
            cv2.createTrackbar("max_%s" % (j), "Trackbars", self.max_values[k], 255, self.tracker_callback)

    def calibrate(self):
        print("please, write the color that you want to calibrate or the name of new one")
        self.color = raw_input("color to calibrate (Enter): ")
        
        if self.color in self.config_data.keys():
            print("existing color selected!")     
        else:
            print("new color selected!")
            self.config_data[self.color] = {"min":{"H":0,"S":0,"V":0}, "max":{"H":255,"S":255,"V":255}}

        self.update_color()
        self.setup_trackbars()
        self.update_trackbar()

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

                center,radius,rect = self.get_rect(cv_image)

                if center != None:
                    
                    img_crop, img_rot = self.crop_rect(cv_image,rect)

                    self.crop_pub.publish(bridge.cv2_to_imgmsg(img_crop, encoding="bgr8"))
                    p = Point()
                    p.x = center[0]
                    p.y = center[1]
                    p.z = radius
                    print(radius)
                    if radius > self.min_radius: 
                        self.ref_pub.publish(p)

            k = cv2.waitKey(5)
            if k == 27 : break  #esc pressed
            elif rospy.get_param('~calibrate',False) and k == ord("s") : self.save_color()

    

def main():
    
    c = color_range()
    c.run()

    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()