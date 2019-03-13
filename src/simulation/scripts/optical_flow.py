#!/usr/bin/env python
# optical flow
from __future__ import print_function

import cv2
import numpy as np


class CameraStabilization():

    feature_params = dict(maxCorners=100,
                          qualityLevel=0.002,
                          minDistance=7,
                          blockSize=7)

    # Parameters for lucas kanade optical flow
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.03))

    def __init__(self, flip=False):

        self.flip = flip

        self.features_drift = None
        self.global_drift = None
        self.feature_positions = None
        self.global_position = None

        # Create some random colors
        self.color = np.random.randint(0, 255, (100, 3))
        self.frame = 0

    # interval format = [y0,yf,x0,xf]
    def resetFeatures(self, initial_img, interval=None, get_corners_only=False):

        self.display_mask = np.zeros_like(
            initial_img)  # mask for movement lines
        self.old_gray = cv2.cvtColor(
            initial_img, cv2.COLOR_BGR2GRAY)  # coverts to gray scale

        self.position_mean = 0  # pixel's positions mean
        search_space = self.old_gray[:]
        if interval != None:
            search_space = self.old_gray[interval[0]
                :interval[1], interval[2]:interval[3]]

        # detect features in initial_img to track
        # p0 contains the last frame feature positions(x,y) - p0.shape = (number of features, 1, 2)
        print("reset")
        self.p0 = cv2.goodFeaturesToTrack(
            search_space, mask=None, **self.feature_params)
        h, w = search_space.shape
        print(self.p0)
        if get_corners_only:
            tl = np.argmax(np.linalg.norm(self.p0, axis=2))
            tr = np.argmax(np.linalg.norm(self.p0 - np.array([0, h]), axis=2))
            bl = np.argmax(np.linalg.norm(self.p0 - np.array([w, 0]), axis=2))
            br = np.argmax(np.linalg.norm(self.p0 - np.array([w, h]), axis=2))
            new_p0 = np.array(
                [self.p0[tl], self.p0[tr], self.p0[bl], self.p0[br]])
            self.p0 = new_p0
        # self.p0 += np.array([interval[2], interval[0]])

        return self.p0

    def update(self, frame, feature_params=feature_params, show=True, precision_cut=10):

        movement = np.zeros(2)
        precise = True  # becomes false if the standard deviation > precision_cut

        if self.flip:
            frame = cv2.flip(frame, 1)

        if self.frame == 0:
            print("1")
            # self.resetFeatures(frame)
            self.frame += 1
        else:
            self.frame += 1
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # p1 contains p0 feature on the new frame positions(x,y)
            p1, st, err = cv2.calcOpticalFlowPyrLK(
                self.old_gray, frame_gray, self.p0, None, **self.lk_params)

            if(type(p1) == 'NoneType'):
                print("frame contains no fetures")
                return False
                # self.resetFeatures(frame)
                # return

            good_new = p1[st == 1]  # select only usable features
            good_old = self.p0[st == 1]

            self.feature_positions = good_new

            if show:
                self.display(frame, good_new, good_old)

            raw_features_drif = good_new - good_old
            # print(raw_features_drif.shape)
            if(raw_features_drif.shape[0] > 1 and raw_features_drif.shape[1] > 1):

                # remove outliers dist_movements
                self.features_drift, total_std = self.cutOutliers(
                    raw_features_drif, num_std=1.2)

                self.global_drift = np.mean(self.features_drift, 0)
                self.global_position = np.mean(good_new, 0)

                self.old_gray = frame_gray.copy()  # update reference gray image
                # update the list of reference points on track (features locations)
                self.p0 = good_new.reshape(-1, 1, 2)

                if(total_std > precision_cut):
                    precise = False
                else:
                    precise = True
            else:
                # forces to have more than 1 feature on the screen
                print("last")
                return False
                # self.resetFeatures(frame)

                # return the imagem movement in pixels and if this value ir precise or not
        return precise

    def cutOutliers(self, list, num_std=2):

        list_mean = np.mean(list, 0)
        std = np.std(list, 0)  # standard deviation
        outlier_cut = num_std * std

        # used to validate or not the frame movement
        total_std = np.average(std)
        # print(total_std)

        # boolean array to exclude random positive movements
        binary_mask_up = list <= list_mean + outlier_cut
        # boolean array to exclude random negative movements
        binary_mask_down = list >= list_mean - outlier_cut

        # boolean array of good movements (on each axis) to consider
        mask_measure = binary_mask_up & binary_mask_down

        # boolean array that leaves only measures with good movements on both axis
        # equivalent to: an and condition for the row
        mask_outlier = np.min(mask_measure, 1)

        return list[mask_outlier], total_std

    def display(self, frame, good_new, good_old):  # draw stuff

        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()

            self.display_mask = cv2.line(
                self.display_mask, (a, b), (c, d), self.color[i].tolist(), 2)
            frame = cv2.circle(frame, (a, b), 5, self.color[i].tolist(), -1)
        pos_mean = good_new.mean(0)
        frame = cv2.circle(
            frame, (pos_mean[0], pos_mean[1]), 10, [255, 0, 0], -1)

        img = cv2.add(frame, self.display_mask)
        cv2.imshow('frame', img)

        return frame

    def get_pos_mean(self):
        return self.feature_positions.mean(0)

    def get_square_center(self):
        return (np.max(self.feature_positions, axis=0) + np.min(self.feature_positions, axis=0)) / 2

    def get_square_shape(self):
        return np.max(self.feature_positions, axis=0) - np.min(self.feature_positions, axis=0)

    def set_p0(self, given_p0):
        self.p0 = given_p0

    def set_initial_img(self, initial_img):
        self.display_mask = np.zeros_like(
            initial_img)  # mask for movement lines
        self.old_gray = cv2.cvtColor(
            initial_img, cv2.COLOR_BGR2GRAY)

    def clearMask(self):
        self.display_mask[:] = 0
        return

    def getDrift(self):
        return self.features_drift, self.global_drift

    def getPosition(self):
        return self.feature_positions, self.global_position


def main():
    print("arquivo com classe do optical flow")
    c = CameraStabilization()

    cap = cv2.VideoCapture(0)
    _, frame = cap.read()

    c.resetFeatures(frame)
    while 1:
        _, frame = cap.read()
        (rows, cols, channels) = frame.shape
        if rows > 1 and cols > 1:
            c.update(frame)
            cv2.imshow("cam", frame)
        k = cv2.waitKey(1)
        if k == 27:
            break


if __name__ == "__main__":
    main()
