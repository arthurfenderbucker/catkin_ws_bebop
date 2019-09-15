#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import time

from cv_bridge import CvBridge, CvBridgeError

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
        self.running = rospy.get_param('~running',True)

        self.image_topic_sub = rospy.Subscriber(
            "cv_detection/rectangle_detector/set_img_topic", String, self.set_image_topic, queue_size=None)
        self.pub_topic_sub = rospy.Subscriber(
            "cv_detection/rectangle_detector/set_pub_topic", String, self.set_pub_topic, queue_size=None)
        self.running_sub= rospy.Subscriber(
            "cv_detection/rectangle_detector/set_runnig_state", Bool, self.set_runninng_state, queue_size=None)
        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)
        self.count_fps = 0
        self.t_0 = 0

    def set_image_topic(self, topic_name):
        self.img_topic = topic_name.data

    def set_pub_topic(self, topic_name):
        self.pub_topic = topic_name.data
        self.ref_pub = rospy.Publisher(self.pub_topic, Point, queue_size=1)

    def set_runninng_state(self,boolean_state):
        self.running = boolean_state.data


    def update(self, image,show=True):

        # convert the image to grayscale, blur it, and find edges
        # in the image
        img = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #------------- contours, area & perpendicular approach -----------------
        for i in range(3):
            gray = cv2.bilateralFilter(gray, 11, 5, 11)
        cv2.imshow("gray bi", gray)
        edged = cv2.Canny(gray, 30, 200, apertureSize=5)
        cv2.imshow("edged", edged)

        # find contours in the edged image, keep only the ones with the higher area
        im2, cnts, _ = cv2.findContours(edged.copy(),
                                        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:30]
        poligons = self.filter_cnts(cnts)
        
        if len(poligons) >0:
            # print("---")
            # print(poligons[0])
            ((x,y), radius) = cv2.minEnclosingCircle(poligons[0])
            
            if show and radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(image, (int(x), int(y)), 3, (0, 0, 255), -1)
                cv2.putText(image,"centroid", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
                # cv2.putText(image,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)

            if show: 
                # image = self.show_rects(image, rects)
                self.show_poligons(image, poligons)
            # rects = self.get_rect(poligons)
            # rects_center = self.get_rect_center(poligons[0])
            # radius = self.get_radius(poligons[0])
            # print(rects_center, radius)
            # rects = cv2.minAreaRect(cnts[0])
            # print(rects)
            # return None, None
            return (x,y),radius
        else:
            return None, None
            
    def filter_cnts(self, cnts_raw):
        cnts = []
        # loop over our contours
        for c in cnts_raw:
            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            # if our approximated contour has four points, then
            # we can assume that we have found our screen
            if len(approx) >= 4 and len(approx) <= 4 and self.perpendicular(approx) < 30 and cv2.contourArea(c)>2000:
                center, size, angle = cv2.minAreaRect(approx)
                if angle < -75 or angle > -15:
                    cnts.append(approx)
        
        return np.array(cnts)

    def get_radius(self,poligons):
        if len(poligons)>0:
            return np.sqrt(np.sum(np.power(poligons.max(axis=0)-poligons.min(axis=0),2)))
        return None
    def get_rect(self,poligons):
        if len(poligons)>0:
            return cv2.minAreaRect(poligons)
        return None
    def get_rect_center(self, poligons):
        if len(poligons)>0:
            return  np.mean([poligons.max(axis=0),poligons.min(axis=0)],axis = (0,1))
        return None
        
    def show_poligons(self, image, poligons):
        try:
            for i in range(len(poligons)):
                cv2.drawContours(
                    image, [poligons[i]], -1, self.colors[i].tolist(), 3)
            cv2.imshow("Janela", image)
            return 1
        except:
            print("erros")
            return 0

    def show_rects(self, image, rects):
        if not rects == None and len(rects)>0:
            # print(tuple(rects))
            rects = np.int0(rects)
            for r in rects:
                cv2.rectangle(image,tuple(r[0,:]),tuple(r[1,:]),(0,0,255),2)
            return image
    def area_and_perp(self, points):
        print(cv2.contourArea(points))
        if cv2.contourArea(points) > 0:
            return cv2.contourArea(points)
        else:
            return 0

    def perpendicular(self, points):
        total_ang = 0
        for i in range(len(points)):
            l = i - 2
            m = i - 1
            if i == 0:
                l = len(points) - 2
                m = len(points) - 1
            elif i == 1:
                l = len(points) - 1
            l1 = points[i][0] - points[m][0]
            l2 = points[m][0] - points[l][0]

            total_ang += abs(self.ang(l1, l2))
        return total_ang / len(points)

    def ang(self, a, b):
        def norm(x):
            mag = np.linalg.norm(x)
            return x / mag
        return np.degrees(np.dot(norm(a), norm(b)))


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
               
                center, radius = self.update(cv_image)
                # print("fps: " +str(1/(time.time() - self.t_0)))
                # self.t_0 = time.time()
                # print("radius: "+str(radius))
                if not center is None:
                    
                    # img_crop, img_rot = self.crop_rect(cv_image,rect)
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


# --------- corners approach --------------
# dst = cv2.cornerHarris(gray, 2, 3, 0.04)
# # result is dilated for marking the corners, not important
# dst = cv2.dilate(dst, None)
# img[dst > 0.01 * dst.max()] = [0, 0, 255]
# cv2.imshow("corners", img)
# ---------- FFT approach ---------------
# f = np.fft.fft2(gray)
# fshift = np.fft.fftshift(f)
# magnitude_spectrum = 20 * np.log(np.abs(fshift))
# magnitude_spectrum = magnitude_spectrum.astype(np.int8)
# # magnitude_spectrum = np.expand_dims(magnitude_spectrum, axis=-1)
# print(magnitude_spectrum.dtype)
# cv2.imshow("FFT", magnitude_spectrum)
# plt.show()
# --------- hogh lines approach ----------

# img = cv2.GaussianBlur(img, (11, 11), 0)
# gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# cv2.imshow("gray", gray_img)
# cv2.multiply(gray_img, np.array([1.5]), gray_img)
# # get edges using laplacian
# laplacian_val = cv2.Laplacian(gray_img, cv2.CV_32F)

# # lap_img = np.zeros_like(laplacian_val, dtype=np.float32)
# # cv2.normalize(laplacian_val, lap_img, 1, 255, cv2.NORM_MINMAX)
# cv2.imshow('laplacian_val', laplacian_val)

# # apply threshold to edges

# ret, laplacian_th = cv2.threshold(
#     laplacian_val, thresh=2, maxval=255, type=cv2.THRESH_BINARY)
# cv2.imshow('laplacian_th', laplacian_th)
# # filter out salt and pepper noise
# laplacian_med = cv2.medianBlur(laplacian_th, 5)
# cv2.imshow('laplacian_med', laplacian_med)
# # cv2.imwrite('laplacian_blur.jpg', laplacian_med)
# laplacian_fin = np.array(laplacian_med, dtype=np.uint8)

# # note this is a horizontal kernel
# # kernel = np.ones((1, 20), np.uint8)
# # d_im = cv2.dilate(laplacian_fin, kernel, iterations=1)
# # e_im = cv2.erode(d_im, kernel, iterations=1)

# # cv2.imshow("e_im", e_im)

# # get lines in the filtered laplacian using Hough lines
# lines = cv2.HoughLines(laplacian_fin, 2, np.pi / 180, 120)

# if lines is not None:
#     for l in lines[:32]:
#         print(l[0])
#         for rho, theta in l:
#             a = np.cos(theta)
#             b = np.sin(theta)
#             x0 = a * rho
#             y0 = b * rho
#             x1 = int(x0 + 1000 * (-b))
#             y1 = int(y0 + 1000 * (a))
#             x2 = int(x0 - 1000 * (-b))
#             y2 = int(y0 - 1000 * (a))
#             # overlay line on original image
#             cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

# # cv2.imwrite('processed.jpg', img)
# cv2.imshow('Window', img)