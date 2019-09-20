#!/usr/bin/env python

import rospy
import numpy as np


from geometry_msgs.msg import Point
from std_msgs.msg import String

def array_to_vec(v):
    vec = Point()
    vec.x = v[0]
    vec.y = v[1]
    vec.z = v[2]
    return vec

def main():
    pub = rospy.Publisher('/control/position_relative', Point, queue_size=1)
    rospy.init_node('zigzag', anonymous=True)
    rate =rospy.Rate(0.125)
    while not rospy.is_shutdown():
        
        rospy.loginfo("[0,0,1]")
        pub.publish(array_to_vec([0,0,1]))

        t = raw_input()
        rospy.loginfo("[0,0,-1]")
        pub.publish(array_to_vec([0,0,-1])
        t = raw_input()
        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

