import cv2
import numpy as np
import rospy
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--video", help="runs cropbycolor on real time webcam video",
                    action="store_true")

parser.add_argument("-i", "--image", help="runs cropbycolor on sample image 'ex.png'",
                    action="store_true")
args = parser.parse_args()

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


def cutflag(img):
    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    except Exception as e:
        gray = img
        pass
    ret, thr = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #thr = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)

    kernel = np.ones((5,5),np.uint8)
    thr = cv2.morphologyEx(thr, cv2.MORPH_CLOSE, kernel)
    thr = cv2.morphologyEx(thr, cv2.MORPH_OPEN, kernel)
    # frame_to_thresh = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower_red = np.array([150,80,135])
    # higher_red = np.array([215,180,255])


    #thr = cv2.inRange(thr, lower_red, higher_red)

    cv2.imshow("Threshold", thr)
    #ret,thr = cv2.threshold(gray,0,127,0)
    #contours = cv2.findContours(thr, 1, 2)
    _, contours, hierarchy = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #print("Contours: "+ str(contours))
    #print(hierarchy)
    if len(contours) > 0:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        cnt = contours[0]
        rect = cv2.minAreaRect(cnt)
        #rect = cv2.maxAreaRect(cnt)
        img_crop, img_rot = crop_rect(img, rect)
        cont = cv2.drawContours(img, cnt, -1, (255, 0, 0), 5)
        cv2.imshow("Cnt", cont)
        cv2.imshow("img_crop", img_crop)

        x,y,w,h = cv2.boundingRect(cnt)

        if (x == 0 or y == 0) and len(contours) > 1:
            cnt = contours[1]
        #cnt = contours[0]
        M = cv2.moments(cnt)
        # Definition of the centroid coordinates
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centroid = (cx, cy)
            cv2.circle(img,centroid,3,(0,255,0),2)

        print(x, y, w, h)
        #cv2.rectangle(img, (x,y), (x+w, y+h), (0,0,255),3)
        print ("rect: " + str(rect[1][1]))
        cv2.rectangle(img, (int(rect[1][0]), int(rect[1][1])), (int(rect[0][0]), int(rect[0][1])), (0,0,255),3)
        cv2.drawContours(img, cnt , -1, (255,0,0), 3)
        cv2.imshow("rec and cnt", img)
        return img_crop

def main():
    if args.video:
        print("Entrou no if")
        cap = cv2.VideoCapture(-1)
        while True:
            rec, img = cap.read()
            if rec is True:
                rectangles = cutflag(img)
                try:
                    cv2.imshow("Rectangles", rectangles)
                except Exception as e:
                    print(e)

            if cv2.waitKey(1) & 0xFF is 27:
                break

if __name__ == "__main__":
    main()
