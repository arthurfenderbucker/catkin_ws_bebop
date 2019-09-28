#!/usr/bin/env python
import numpy as np
import cv2
from matplotlib import pyplot as plt
import rospkg
from pyzbar.pyzbar import pyzbar
rospack = rospkg.RosPack()

# config_path = str(rospack.get_path('cv_detection')+'/config/feature_config.json')
images_path = str(rospack.get_path('cv_detection')+'/imgs/rectangle/cropped/')


def display(im, decodedObjects):
 
  # Loop over all decoded objects
  for decodedObject in decodedObjects: 
    points = decodedObject.polygon
 
    # If the points do not form a quad, find convex hull
    if len(points) > 4 : 
      hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
      hull = list(map(tuple, np.squeeze(hull)))
    else : 
      hull = points;
     
    # Number of points in the convex hull
    n = len(hull)
 
    # Draw the convext hull
    for j in range(0,n):
      cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)


cap = cv2.VideoCapture(0)
while True:
    ret, image = cap.read()
    if ret:
        decodedObjects = pyzbar.decode(image)
        display(image, decodedObjects)
        for obj in decodedObjects:
            print('Type : ', obj.type)
            print('Data : ', obj.data,'\n')
            x,y,w,h= obj.rect
            print(x,y)
        cv2.imshow("result", image)
    k = cv2.waitKey(1)
    if k == 27 :
        break

cv2.destroyAllWindows()