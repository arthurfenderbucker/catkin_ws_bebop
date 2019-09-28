#!/usr/bin/env python

# sudo pip install zbar
# sudo pip install pyzbar

# reference https://www.pyimagesearch.com/2018/05/21/an-opencv-barcode-and-qr-code-scanner-with-zbar/
from __future__ import print_function
import rospy
import cv2
import gi
import numpy as np
from qrtools import QR
#from drone_video import Video
import pyzbar.pyzbar as pyzbar
import cv_bridge
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
def decode(im) :
  # Find barcodes and QR codes
  decodedObjects = pyzbar.decode(im)
  

  # Print results
  for obj in decodedObjects:
    print('Type : ', obj.type)
    print('Data : ', obj.data,'\n')

  return decodedObjects


# Display barcode and QR code location
def display(im, decodedObjects):

  # Loop over all decoded objects
  for decodedObject in decodedObjects:
    points = decodedObject.polygon

    # If the points do not form a quad, find convex hull
    if len(points) > 4 :
      hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
      hull = list(map(tuple, np.squeeze(hull)))
    else:
      hull = points;

    # Number of points in the convex hull
    n = len(hull)

    # Draw the convext hull
    for j in range(0,n):
      cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

  # Display results
  cv2.imshow("Results", im);

def image_callback(self, image):
    try:
        self.cv_image = self.bridge.imgmsg_to_cv2(image, "mono8")
        #self.gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.gray = self.cv_image
    except CvBridge as e:
        print(e)
    cv2.waitKey(3)

if __name__ == '__main__':
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/bebop/image_raw", Image, image_callback)
    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame()

        cv2.imshow('frame', frame)

        decodedObject = decode(frame)
        display(frame, decodedObject)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
