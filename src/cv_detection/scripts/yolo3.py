#!/usr/bin/env python2
import sys
import argparse
print(sys.version)
print(sys.argv)
print(sys.api_version)
from yolo3. yolo import YOLO, detect_video

from PIL import Image
import cv2
import numpy as np
from optical_flow import CameraStabilization as camStab


def detect_img(yolo):

    cap = cv2.VideoCapture(0)
    mean_box_cord = np.array([0, 0, 0, 0])
    frame = 0

    # cam_stab = camStab(flip=True)
    while True:
        ret, image = cap.read()
        # img = input('Input image filename:')

        if ret:

            image_mat = Image.fromarray(image)
            r_image, out_boxes, out_score = yolo.detect_image(image_mat)
            if out_score.shape[0] > 0:
                m_box = out_boxes.mean(0)

                if frame == 0:
                    mean_box_cord = m_box[:]

                else:
                    mean_box_cord = mean_box_cord * 0.8 + m_box[:] * 0.2

                img = cv2.circle(
                    r_image, (mean_box_cord[1], mean_box_cord[0]), 5, (200, 0, 0), -1)
                img = cv2.circle(
                    img, (mean_box_cord[3], mean_box_cord[2]), 5, (200, 0, 0), -1)
                img = cv2.circle(img, (int((mean_box_cord[3] + mean_box_cord[1]) / 2), int(
                    (mean_box_cord[0] + mean_box_cord[2]) / 2)), 5, (0, 200, 0), -1)

                frame += 1
                # print(type(r_image))
                cv2.imshow("boxed", img)
                cv2.imshow(
                    "main_box", image[mean_box_cord[0]:mean_box_cord[2], mean_box_cord[1]:mean_box_cord[3]])

        else:
            print("none")

        k = cv2.waitKey(30) & 0xff
        if (k == 27):
            break
    yolo.close_session()
    cv2.destroyAllWindows()
    cap.release()


FLAGS = None

if __name__ == '__main__':

    print("Image detection mode")
    detect_img(YOLO())
