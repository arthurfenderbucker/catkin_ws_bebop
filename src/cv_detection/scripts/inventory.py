#!/usr/bin/env python2.7

import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Bool
from cv_bridge import CvBridge, CvBridgeError
import pytesseract
import time
import cv_common
import json
from pyzbar.pyzbar import pyzbar
import rospkg
rospack = rospkg.RosPack()
color_config_path = str(rospack.get_path('cv_detection')+'/config/color_config.json')
inventory_path = str(rospack.get_path('cv_detection')+'/config/inventory.json')


class Inventory:
    running = False
    def __init__(self,color="dark_green",image_topic="/bebop/image_raw", pub_topic="/cv_detection/color_range/detection"):


        rospy.init_node('alpha', anonymous=True)

        with open(color_config_path) as json_data_file:
            self.config_data = json.load(json_data_file)


        self.img_topic = rospy.get_param('~image_topic',image_topic)
        self.pub_topic = rospy.get_param('~pub_topic',pub_topic)
        self.running = rospy.get_param('~running',True)

        self.bridge = CvBridge()
        self.running_sub= rospy.Subscriber(
            "cv_detection/inventory/set_runnig_state", Bool, self.set_running_state, queue_size=None)
        self.running_sub= rospy.Subscriber(
            "cv_detection/inventory/read_tag", Empty, self.read_tag, queue_size=None)
        self.running_sub= rospy.Subscriber(
            "cv_detection/inventory/stop_reading_qr", Empty, self.stop_reading_qr, queue_size=None)
        


        self.min_values = self.config_data[color]["min"].values()
        self.max_values = self.config_data[color]["max"].values()

        self.inventory_data = {}
        self.shelf_slot = ''
        self.qrs = []
        self.count_tag_reads = 20
        self.tag_reads = []
        self.reading_qr = False
        print("done")
    # ========================== topics callbacks ==========================
    def set_running_state(self,data):#Bool
        self.running = data.data
    
    def crop_rect(self, img, rect):
        # get the parameter of the small rectangle
        if len(list(rect)) == 4:
            angle = 0
            x, y, w, h = rect
            center, size= (x+int(w/2),y+int(h/2)),(w,h)
        else:    
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

        (h, w) = img_crop.shape[:2]
        if h>w: #if in the wrong orientation
            #rotated 90 degrees 
            M = cv2.getRotationMatrix2D(center, 90, 1)
            img_crop = cv2.warpAffine(img_crop, M, (h, w)) 

        return img_crop, img_rot

    def detect(self, image):
        rospy.loginfo("Detecting Alphanumeric Codes!")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        et, im_th = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY_INV)
        
        
        # # se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        # # gray = cv2.morphologyEx(image, cv2.MORPH_CLOSE, se)
        # f = 300/im_th.shape[0]
        # im_th = cv2.resize(im_th,(int(f*im_th.shape[1]),300),interpolation = cv2.INTER_AREA)
        cv2.imshow("imageeeeeeeee", im_th)
        # OCR

        config = '--oem 3 --psm 10 -c tessedit_char_whitelist=0123456789AB'#CDEFGHIJKLMNOPQRSTUVWXYZ'
        text = pytesseract.image_to_string(im_th, config=config)
        print(text)
        return text
    

    def get_color_rect(self,image, show = False):

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
            # rect = cv2.boundingRect(c)

            # return image[rect[0]:rect[2],rect[1]:rect[3]]
            # print(rect)
            img_crop, img_rot = self.crop_rect(image, rect)
            return img_crop, rect
        return None,None
    def get_rect(self, image):
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
        
        if len(poligons) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = poligons[0]
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            rect = cv2.minAreaRect(c)
            # rect = cv2.boundingRect(c)

            # return image[rect[0]:rect[2],rect[1]:rect[3]]
            # print(rect)
            img_crop, img_rot = self.crop_rect(image, rect)
            return img_crop, rect
        else:
            return None, None


    def read_tag(self,empty_data):
        self.count_tag_reads = 0
        self.tag_reads = []
        print("read_tag")

    def stop_reading_qr(self,data):#Bool
        self.reading_qr = False

    def most_frequent(self, List): 
        counter = 0
        num = List[0] 
        
        for i in List: 
            curr_frequency = List.count(i) 
            if(curr_frequency> counter): 
                counter = curr_frequency 
                num = i 
    
        return num 
    def save_inventory(self):
        with open(inventory_path, 'w') as json_data_file:
            json.dump(self.inventory_data, json_data_file)
    def display(self,im, decodedObjects):

        # Loop over all decoded objects
        for decodedObject in decodedObjects:
            points = decodedObject.polygon

            # If the points do not form a quad, find convex hull
            if len(points) > 4 :
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points

                # Number of points in the convex hull
                n = len(hull)

            # Draw the convext hull
            for j in range(0,n):
                cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                data = rospy.wait_for_message(self.img_topic, Image)
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                except CvBridgeError as e:
                    print(e)

                (rows, cols, channels) = cv_image.shape
                if not (cols > 60 and rows > 60):  # returns if data have unvalid shape
                    continue

                cropped_image,color_rect= self.get_rect(cv_image)
                if not color_rect is None:
                    (color_rect_x,color_rect_y),(color_rect_w,color_rect_h),ang = color_rect

                    # check if the color is still in scene with the proper size
                    if color_rect_w > 30 and color_rect_h > 20:
                        tag_y = color_rect_y
                    else:
                        tag_y = cv_image.shape[0]
                else:
                    tag_y = cv_image.shape[0]

                #detect alpha num (Tag)
                num_reads = 20
                #dont look for tags all the time and when looking, consider only tags biger than ...
                if self.count_tag_reads < num_reads and not cropped_image is None and cropped_image.shape[0] > 20 and cropped_image.shape[1] > 20: 
                    text = self.detect(cropped_image)
                    self.count_tag_reads += 1
                    print(self.count_tag_reads)

                    if len(text) == 3 and not text in self.inventory_data.keys():
                        self.tag_reads+=[str(text)]
                    if self.count_tag_reads == num_reads and len(self.tag_reads) > 0:

                        self.shelf_slot = self.most_frequent(self.tag_reads) #takes only the most reliable reads
                        print("=====TAG======")
                        print(self.shelf_slot)
                        print("==============")
                        print(self.tag_reads)

                        self.inventory_data[self.shelf_slot] = []
                        self.reading_qr = True
                        print('ok')
                
                #detect QR
                
                if self.reading_qr and not self.shelf_slot == '': # a tag habe been detected
                    decodedObjects = pyzbar.decode(cv_image)
                    cv2.line(cv_image, (0,int(tag_y)), (0,int(tag_y)), (255,0,0), 3)
                    self.display(cv_image,decodedObjects)
                    cv2.imshow("image",cv_image)
                    for obj in decodedObjects:
                        x,y,w,h = obj.rect
                        print("d ",)
                        if y < tag_y-15: #only consider boxes above the shelf slot tag
                            print("above")
                            if not obj.data in self.inventory_data[self.shelf_slot]: #dont repeat qrs
                                self.inventory_data[self.shelf_slot]+=[obj.data]
                                self.save_inventory()
                                print('Type : ', obj.type)
                                print('Data : ', obj.data,'\n')
                                print("saved")

            k = cv2.waitKey(1)
            if k == 27 : break  #esc pressed

def main():

    print("inti ")
    detecter = Inventory()
    detecter.run()


    cv2.destroyAllWindows()


main()
