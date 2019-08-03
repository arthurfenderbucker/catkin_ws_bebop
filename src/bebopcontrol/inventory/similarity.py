import cv2
import numpy as np
from flag import cutflag
img_a = '/home/caio/bebop_ws/src/bebopcontrol/inventory/flag1.jpeg'
img_b = '/home/caio/bebop_ws/src/bebopcontrol/inventory/flag4.jpeg'

img_a = cv2.imread(img_a, cv2.IMREAD_GRAYSCALE)
img_a = cutflag(img_a)
img_b = cv2.imread(img_b, cv2.IMREAD_GRAYSCALE)
img_b = cutflag(img_b)
sift = cv2.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(img_a, None)
kp2, des2 = sift.detectAndCompute(img_b, None)
a = cv2.drawKeypoints(img_a, kp1, None)
# cv2.imshow("keypoints sift", a)

surf = cv2.xfeatures2d.SURF_create()
keypoints_surf, descriptors = surf.detectAndCompute(img_a, None)
b = cv2.drawKeypoints(img_a, keypoints_surf, None)
# cv2.imshow("keypoints_surf", b)

orb = cv2.ORB_create()#nfeatures=150
kp1, des1 = orb.detectAndCompute(img_a, None)
kp2, des2 = orb.detectAndCompute(img_b, None)
key1 = cv2.drawKeypoints(img_a, kp1, None)
key2 = cv2.drawKeypoints(img_b, kp2, None)
# cv2.imshow("keypoints image a", key1)
# cv2.imshow("keypoints image b", key2)

### Frobenius norm of difference image
img_br = cv2.resize(img_b, (img_a.shape[1], img_a.shape[0]))
e = np.linalg.norm(img_a - img_br, 'fro')
print("Frobenius norm: " + str(e))

# Brute Force Matching - compare between all descriptors
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True) #crosscheck - only the best Match
matches = bf.match(des1, des2)
matches = sorted(matches, key=lambda x:x.distance)

match_img = cv2.drawMatches(img_a, kp1, img_b, kp2, matches[:10], None)
cv2.imshow("Matches", match_img)

sum=0
for m in matches[:10]:
    sum += m.distance
    print(m.distance)
print ("Sum = " + str(sum))

print(len(matches))
cv2.waitKey(0)
