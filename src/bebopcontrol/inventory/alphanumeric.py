#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pytesseract
import time

class DetectAlphanumeric:
    def __init__(self):
        self.frame = 0
        self.bridge = CvBridge()
        self.gray = np.zeros((256, 256, 1), dtype = "uint8")
        self.cv_image =  np.zeros((256, 256, 1), dtype = "uint8")
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback, queue_size=2)

    def image_callback(self, image):
        print("callback")
        if self.frame % 3 == 0:
            self.frame += 1
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(image, "mono8")
                self.gray = self.cv_image
                cv2.imshow("Camera", self.gray)
                self.detect()
            except CvBridgeError as e:
                print("Error")
                print (e)
            cv2.waitKey(0)
        else:
            self.frame += 1

    def detect(self):
        print("Detect")
        # blur = cv2.GaussianBlur(self.gray,(3,3),0)
        ret3, thr = cv2.threshold(self.cv_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        #cv2.imshow("Camera", self.gray)
        cv2.imshow("Threshold",thr)
        temp = pytesseract.image_to_string(self.gray)

        i=0
        j=0
        while i < 3 and j < 30:
            j += 1
            data = pytesseract.image_to_string(self.gray)
            if data == temp:
                i += 1
            else:
                temp = data
                i=0
        self.text = data

        print(self.text)
        return self.text


def main():
    rospy.init_node('DetectAlphanumeric', anonymous=True)
    detecter = DetectAlphanumeric()
    image_sub = rospy.Subscriber("/bebop/image_raw", Image, detecter.image_callback, queue_size=2)
    rate = rospy.Rate(20)
    while not (cv2.waitKey(1) & 0xFF == ord('q')):
        print('entrou no while')
        rate.sleep()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting Down')
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
