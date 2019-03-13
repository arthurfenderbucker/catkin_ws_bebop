#!/usr/bin/env python
import numpy as np
import cv2
from matplotlib import pyplot as plt


class detect_window(object):
    """docstring for detect_window"""

    def __init__(self):
        super(detect_window, self).__init__()
        self.colors = np.random.randint(0, 255, (10, 3))

    def update(self, image):

        # image = cv2.resize(image_in, (300, 300), interpolation=cv2.INTER_CUBIC)
        # convert the image to grayscale, blur it, and find edges
        # in the image
        img = image.copy()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray", gray)

        #------------- contours, area & perpendicular approach -----------------
        gray = cv2.bilateralFilter(gray, 11, 17, 17)
        cv2.imshow("gray bi", gray)
        edged = cv2.Canny(gray, 30, 200, apertureSize=5)
        cv2.imshow("edged", edged)

        kernel = np.ones((15, 1), np.uint8)
        d_im = cv2.dilate(edged, kernel, iterations=1)
        e_im = cv2.erode(d_im, kernel, iterations=1)
        # kernel = np.ones((1, 15), np.uint8)
        # d_im = cv2.dilate(e_im, kernel, iterations=1)
        # e_im = cv2.erode(d_im, kernel, iterations=1)
        cv2.imshow("edged_d_e", e_im)
        edged = e_im.copy()

        # find contours in the edged image, keep only the largest
        # ones, and initialize our screen contour
        im2, cnts, _ = cv2.findContours(edged.copy(),
                                        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:30]
        # cnts = sorted(cnts, key=area_and_perp, reverse=False)[:30]
        # print(cnts)
        screenCnt = []
        # loop over our contours
        for c in cnts:
            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            # if our approximated contour has four points, then
            # we can assume that we have found our screen
            if len(approx) >= 4 and len(approx) <= 7 and self.perpendicular(approx) < 30:
                screenCnt.append(approx)

        try:
            # screenCnt = sorted(screenCnt, key=area_and_perp, reverse=False)
            # cv2.drawContours(image, [screenCnt[0]], -1,
            #                  self.colors[0].tolist(), 3)
            for i in range(len(screenCnt)):
                cv2.drawContours(
                    image, [screenCnt[i]], -1, self.colors[i].tolist(), 3)
            cv2.imshow("Janela", image)
        except:
            print("erros")

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


# test = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
# print(perpendicular(test))
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    d = detect_window()
    while (1):
        _, image = cap.read()

        d.update(image)
        k = cv2.waitKey(30) & 0xff
        if (k == 27):
            break
    cv2.destroyAllWindows()
    cap.release()
