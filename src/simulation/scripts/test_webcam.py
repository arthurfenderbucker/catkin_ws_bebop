
import cv2 as cv
# print(cv.__version__)

cap = cv.VideoCapture(0)
while 1:
    _, frame = cap.read()

    cv.imshow("img",frame)
    k = cv.waitKey(1)
    if k == 27:
        break
