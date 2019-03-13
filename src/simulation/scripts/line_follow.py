#!/usr/bin/env python
import numpy as np
import cv2
from matplotlib import pyplot as plt


class line_follow(object):
    """docstring for detect_window"""

    def __init__(self):
        super(line_follow, self).__init__()
        self.colors = np.random.randint(0, 255, (10, 3))

    def update(self, image, n_lines=8):
        if not hasattr(self, 'h'):
            self.h, self.w, self.d = image.shape
            self.cx, self.cy = self.w/2, self.h/2

        img = image.copy()
        img = cv2.GaussianBlur(img, (11, 11), 0)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray", gray_img)
        cv2.multiply(gray_img, np.array([1.5]), gray_img)
        # get edges using laplacian
        laplacian_val = cv2.Laplacian(gray_img, cv2.CV_32F)

        # lap_img = np.zeros_like(laplacian_val, dtype=np.float32)
        # cv2.normalize(laplacian_val, lap_img, 1, 255, cv2.NORM_MINMAX)
        cv2.imshow('laplacian_val', laplacian_val)

        # apply threshold to edges

        ret, laplacian_th = cv2.threshold(
            laplacian_val, thresh=1, maxval=255, type=cv2.THRESH_BINARY)
        cv2.imshow('laplacian_th', laplacian_th)
        # filter out salt and pepper noise
        laplacian_med = cv2.medianBlur(laplacian_th, 5)
        cv2.imshow('laplacian_med', laplacian_med)
        # cv2.imwrite('laplacian_blur.jpg', laplacian_med)
        laplacian_fin = np.array(laplacian_med, dtype=np.uint8)

        # note this is a horizontal kernel
        # kernel = np.ones((1, 20), np.uint8)
        # d_im = cv2.dilate(laplacian_fin, kernel, iterations=1)
        # e_im = cv2.erode(d_im, kernel, iterations=1)

        # cv2.imshow("e_im", e_im)

        # get lines in the filtered laplacian using Hough lines
        lines = cv2.HoughLines(laplacian_fin, 2, np.pi / 180, 120)

        if lines is not None:
            for l in lines[:n_lines]:
                self.draw_line(l, img)
        grad_x, grad_y = self.avared_grad(lines[:n_lines])
        h, w = self.h, self.w
        cv2.line(img, (int(w / 2), int(h / 2)), (int(w / 2 + grad_x *
                                                     150), int(h / 2 + grad_y * 150)), (0, 0, 0), 2)

        cx_new, cy_new = self.rope_center(lines[:n_lines])
        alpha = 0.2
        self.cx = alpha*self.cx + (1-alpha)*cx_new
        self.cy = alpha*self.cy + (1-alpha)*cy_new


        cv2.circle(img, (int (self.cx), int(self.cy)), 10, (0, 255, 0))
        cv2.imshow('Window', img)
        return lines[:n_lines]

    def rope_center(self, lines):  # rope center
        gx,gy = 0,0
        for l in lines:
            for rho, theta in l:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                h, w = self.h, self.w
                x,y = self.line_intersection([[x1,y1],[x2,y2]],[[w/2,h/2],[w/2+a,h/2+b]])
                gx+=x
                gy+=y
        return gx/len(lines), gy/len(lines)

    def avared_grad(self, lines):
        a = np.cos(lines[:, 0, 1])
        b = np.sin(lines[:, 0, 1])
        a[b < 0] = -a[b < 0]
        gx,gy = -np.mean(b),np.mean(a)
        return gx if (gy < 0) else -gx, gy if (gy < 0) else - gy

    def draw_line(self, l, img, color=(0, 255, 0)):
        for rho, theta in l:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            # overlay line on original image
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
            h, w = self.h, self.w
            x,y = self.line_intersection([[x1,y1],[x2,y2]],[[w/2,h/2],[w/2+a,h/2+b]])
            cv2.circle(img, (int (x), int(y)), 5, (255, 0, 0))
        return

    def line_intersection(self,line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
           raise Exception('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

    def contourLenght(self, cnts):
        return cv2.arcLength(cnts, False)


# test = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
# print(perpendicular(test))
if __name__ == "__main__":
    cap = cv2.VideoCapture(-1)
    d = line_follow()
    while (1):
        _, image = cap.read()

        d.update(image)
        k = cv2.waitKey(30) & 0xff
        if (k == 27):
            break
    cv2.destroyAllWindows()
    cap.release()

    # if lines is not None:
    #         for l in lines[:8]:
    #             pass
    #             # self.draw_line(l, img)

    #     # self.draw_line(np.mean(lines[:8], axis=0), img, color=(255, 0, 0))
    #     cv2.imshow('Window', img)

    #     return lines[:8]

    # def draw_line(self, l, img, color=(0, 255, 0)):
    #     for rho, theta in l:
    #         a = np.cos(theta)
    #         b = np.sin(theta)
    #         x0 = a * rho
    #         y0 = b * rho
    #         x1 = int(x0 + 1000 * (-b))
    #         y1 = int(y0 + 1000 * (a))
    #         x2 = int(x0 - 1000 * (-b))
    #         y2 = int(y0 - 1000 * (a))
    #         # overlay line on original image
    #         cv2.line(img, (x1, y1), (x2, y2), color, 2)
