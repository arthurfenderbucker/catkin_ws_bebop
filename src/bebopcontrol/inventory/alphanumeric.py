#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pytesseract
import time

def crop_rect(img, rect):
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


class DetectAlphanumeric:
    def __init__(self):
        self.frame = 0
        self.bridge = CvBridge()
        self.gray = np.zeros((256, 256, 1), dtype = "uint8")
        self.cv_image =  np.zeros((256, 256, 1), dtype = "uint8")
        self.cropped_img = np.zeros((256, 256, 1), dtype = "uint8")
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback, queue_size=2)

        self.lower_red = np.array([153,81,136])
        self.higher_red = np.array([214,173,255])

    def image_callback(self, image):
        global rate
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            #cv2.imshow("Camera Image", self.cv_image)
        except CvBridgeError as e:
            print (e)
        print("Callback!")
        for i in range(100):
            rate.sleep()

        # if self.frame < 5:
        #     self.frame += 1
        # else:
        #     self.cropped_img = self.preprocessing()
        #     self.text = self.detect()
        #     self.frame=0


    def detect(self):
        rospy.loginfo("Detecting Alphanumeric Codes!")
        gray = cv2.cvtColor(self.cropped_img, cv2.COLOR_BGR2GRAY)
        ret3, thr = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        #cv2.imshow("Threshold + Crop",thr)
        text = pytesseract.image_to_string(self.preprocessing())
        # temp = pytesseract.image_to_string(self.preprocessing())
        #
        # i=0
        # j=0
        # while i < 3 and j < 30:
        #     j += 1
        #     data = pytesseract.image_to_string(gray)
        #     if data == temp:
        #         i += 1
        #     else:
        #         temp = data
        #         i=0
        # self.text = data

        rostopic.loginfo(self.text)
        return self.text

    def preprocessing(self):
        global rate
        center = None
        #print(self.cv_image)
        frame_to_thresh = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(frame_to_thresh, self.lower_red, self.higher_red)

        # ignore random noise and merge very close areas
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        while not len(cnts) > 0:
            frame_to_thresh = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            thresh = cv2.inRange(frame_to_thresh, self.lower_red, self.higher_red)

            # ignore random noise and merge very close areas
            kernel = np.ones((5,5),np.uint8)
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
            rate.sleep()
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
        return img_crop


rospy.init_node('DetectAlphanumeric', anonymous=True)
rate = rospy.Rate(20)
def main():
    # rospy.init_node('DetectAlphanumeric', anonymous=True)
    # rate = rospy.Rate(20)
    detecter = DetectAlphanumeric()
    image_sub = rospy.Subscriber("/bebop/image_raw", Image, detecter.image_callback, queue_size=2)
    while not (cv2.waitKey(1) & 0xFF == ord('q')):
        cv2.imshow("Camera Image", detecter.cv_image)

        cv2.imshow("Cropped image",detecter.cropped_img)
        rate.sleep()
        # try:
        #     rospy.spin()
        #     print('Shutting Down')
        #     break
        # except KeyboardInterrupt:
        #     print('Shutting Down')
        #     break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
