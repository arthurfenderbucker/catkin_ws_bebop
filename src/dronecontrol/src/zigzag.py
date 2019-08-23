#!/usr/bin/env python

import rospy
import numpy as np

from dronecontrol.msg import Vector3D
from std_msgs.msg import String

def array_to_vec(v):
    vec = Vector3D()
    vec.x = v[0]
    vec.y = v[1]
    vec.z = v[2]
    return vec

def main():
    pub = rospy.Publisher('/control/position', Vector3D, queue_size=1)
    rospy.init_node('zigzag', anonymous=True)
    rate =rospy.Rate(0.25)
    while not rospy.is_shutdown():
        
        rospy.loginfo("[0,0,1]")
        pub.publish(array_to_vec([0,0,1]))
        rate.sleep()
        rospy.loginfo("[0,1,1]")
        pub.publish(array_to_vec([0,1,1]))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

