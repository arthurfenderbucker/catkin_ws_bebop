
#!/usr/bin/env python2.7
import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pytesseract

class DetectAlphanumeric:
    def __init__(self):
        self.frame = 0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)

    def image_callback(self, image):
        if self.frame % 3 == 0:
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(image, "mono8")
                #self.gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                self.gray = self.cv_image
                self.detect()

            except CvBridgeError as e:
                print (e)
            cv2.waitKey(3)
        self.frame += 1

    def detect(self):
        blur = cv2.GaussianBlur(self.gray,(3,3),0)
        ret3, th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        cv2.imshow("Camera", self.cv_image)
        cv2.imshow("Threshold",th3)
        text = pytesseract.image_to_string(th3)
        boxes = pytesseract.image_to_string(th3).out_boxes
        print(text)
        return text


def main():
    detecter = DetectAlphanumeric()
    rospy.init_node('DetectAlphanumeric', anonymous=True)
    rate = rospy.Rate(20)
    while not (cv2.waitKey(1) == 27 & 0xFF == ord('q')):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting Down')
            break
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    rate.sleep()

if __name__ == "__main__":
    main()
