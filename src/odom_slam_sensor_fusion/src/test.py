#!/usr/bin/env python
import numpy as np
from sklearn.linear_model import LinearRegression 
import time 
from geometry_msgs.msg import Point, Pose
import ros_numpy

# import scipy
def tf(odom_coord, slam_coord):
    
    x = np.vstack((odom_coord,odom_coord,odom_coord,odom_coord))+np.eye(4,3 )
    y = np.vstack((slam_coord,slam_coord,slam_coord,slam_coord))+np.eye(4,3 )
    odom_coord_expanded = np.hstack([x,np.ones((len(x),1))])
    # print(odom_coord_expanded)
    #yeah... this is kind of a laizy approach...
    reg = LinearRegression().fit(x, y)
    return np.hstack([reg.coef_,np.expand_dims(reg.intercept_,axis=1)])

pose1 = Pose()
pose1.position.x = 1
pose1.position.y = 0
pose1.position.z = 0
pose1.orientation.x = 0
pose1.orientation.y = 0.707
pose1.orientation.z = 0
pose1.orientation.w = 0.707

pose2 = Pose()
pose2.position.x = 0
pose2.position.y = 0
pose2.position.z = 0
pose2.orientation.x = 0
pose2.orientation.y = 0.707
pose2.orientation.z = 0
pose2.orientation.w = 0.707

orb_pose3 = Pose()
orb_pose3.position.x = 1
orb_pose3.position.y = 0
orb_pose3.position.z = 0
orb_pose3.orientation.x = 0
orb_pose3.orientation.y = 0
orb_pose3.orientation.z = 0
orb_pose3.orientation.w = 0

orb_pose1 = Pose()
orb_pose1.position.x = 2
orb_pose1.position.y = 0
orb_pose1.position.z = 3
orb_pose1.orientation.x = 0
orb_pose1.orientation.y = 0
orb_pose1.orientation.z = 0
orb_pose1.orientation.w = 1

orb_pose2 = Pose()
orb_pose2.position.x = 1
orb_pose2.position.y = 0
orb_pose2.position.z = 3
orb_pose2.orientation.x = 0
orb_pose2.orientation.y = 0
orb_pose2.orientation.z = 0
orb_pose2.orientation.w = 1

c1 = ros_numpy.numpify(pose1)
c2 = ros_numpy.numpify(pose2)
orb_c1 = ros_numpy.numpify(orb_pose1)
orb_c2 = ros_numpy.numpify(orb_pose2)
orb_c3 = ros_numpy.numpify(orb_pose3)
print(c1)
print(c2)


dist_c = np.sqrt(np.dot(c1[:3,3]-c2[:3,3],c1[:3,3]-c2[:3,3]))
dist_orb = np.sqrt(np.dot(orb_c1[:3,3]-orb_c2[:3,3],orb_c1[:3,3]-orb_c2[:3,3]))
scale_factor = dist_c/dist_orb

# scale_factor_m = np.eye([4,4])
# scale_factor_m[3,3] = 0
# scale_factor_m *= scale_factor
# scale_factor_m +=1
scale_factor_m = np.ones([4,4])
scale_factor_m[:3,3] = scale_factor

# orb_pose1 = ros_numpy.msgify(Pose,orb_c1)
# orb_pose2 = ros_numpy.msgify(Pose,orb_c2)

# orb_pose1.position.x *= scale_factor
# orb_pose1.position.y *= scale_factor
# orb_pose1.position.z *= scale_factor

# orb_pose2.position.x *= scale_factor
# orb_pose2.position.y *= scale_factor
# orb_pose2.position.z *= scale_factor

# orb_c1 = ros_numpy.numpify(orb_pose1)
# orb_c2 = ros_numpy.numpify(orb_pose2)

print(scale_factor_m)
# tfM_1 = np.dot(c1,np.linalg.inv(orb_c1))
# tfM_2 = np.dot(c2,np.linalg.inv(orb_c2))
tfM_1 = np.dot(c1,np.linalg.inv(orb_c1))
tfM_2 = np.dot(c2,np.linalg.inv(orb_c2))

print("---")
print(tfM_1)
print(tfM_2)
print("---")
print(np.dot(tfM_1,orb_c1))
print(np.dot(tfM_2,orb_c2))
print("-------------------")
print(np.dot(tfM_1,orb_c3))
print(np.dot(tfM_2,orb_c3))
# print(np.dot(tfM_1,c1))
# print(np.dot(tfM_2,c2))

# reg = LinearRegression().fit([orb_c1,orb_c2], [c1,c2])
# print("---reg---")
# print(reg.coef_)
# print(reg.intercept_)

# x = np.array([5-1,3-1,1-1])
# y = np.array([5,3,1])
# t = time.time()
# for i in range(1000):
# M = tf(x,y)
# print(time.time()-t)
# x = np.array([2+3,1+3,-3+3,1])
# x2 = np.array([1,2,-3,1])
# x = np.array([5-1,3-1,1-1,1])
# print(np.dot(x,M.T))
# print(np.dot(x2,M.T))