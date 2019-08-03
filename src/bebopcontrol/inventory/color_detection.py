import cv2
import argparse
import numpy as np

colors = np.random.randint(0, 255, (10, 3))

def callback(): #trackbar callback
    pass
def get_arguments():

    ap = argparse.ArgumentParser()
    ap.add_argument('-f', '--filter', required=False, default="HSV",
                    help='Range filter. RGB or HSV')
    ap.add_argument('-w', '--webcam', required=False,
                    help='Use webcam', default="HSV", action='store_true')
    args = vars(ap.parse_args())

    if not args['filter'].upper() in ['RGB', 'HSV']:
        ap.error("Please speciy a correct filter.")

    return args

def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)
    init_val = [[153,81,136],[214,173,255]] #red hsv
    #init_val = [[71,99,105],[145,234,255]]   #blue


    for w,i in enumerate(["MIN", "MAX"]):
        v = 0 if i == "MIN" else 255

        for k,j in enumerate(range_filter):
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", init_val[w][k], 255, callback)


def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)
    return values

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

def main():
    args = get_arguments()

    range_filter = args['filter'].upper()

    camera = cv2.VideoCapture(0)

    setup_trackbars(range_filter)

    while True:
        if args['webcam']:
            ret, image = camera.read()

            if not ret:
                break

            if range_filter == 'RGB':
                frame_to_thresh = image.copy()
            else:
                frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        # ignore random noise and merge very close areas
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
            rect = cv2.minAreaRect(c)
            img_crop, img_rot = crop_rect(image, rect)
            cv2.imshow("rect",img_crop)
            print(rect)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(image,[box],0,(0,0,255),2)

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(image, center, 3, (0, 0, 255), -1)
                cv2.putText(image,"centroid", (center[0]+10,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)
                cv2.putText(image,"("+str(center[0])+","+str(center[1])+")", (center[0]+10,center[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255),1)


        # show the frame to our screen
        cv2.imshow("Original", image)
        cv2.imshow("Thresh", thresh)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF is 27:
            break



def area_and_perp( points):
    print(cv2.contourArea(points))
    if cv2.contourArea(points) > 0:
        return cv2.contourArea(points)
    else:
        return 0

def perpendicular( points):
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

        total_ang += abs(ang(l1, l2))
    return total_ang / len(points)

def ang( a, b):
    def norm(x):
        mag = np.linalg.norm(x)
        return x / mag
    return np.degrees(np.dot(norm(a), norm(b)))


if __name__ == '__main__':
    main()
