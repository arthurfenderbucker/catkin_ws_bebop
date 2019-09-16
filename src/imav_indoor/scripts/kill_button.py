#!/usr/bin/env python
from std_msgs.msg import String, Empty, Bool
import rospy
import os
import rospkg

rospack = rospkg.RosPack()
killing_file_path = str(rospack.get_path('imav_indoor')+"/../../kill_bebop.txt")
print(killing_file_path)
def main():
    pub = rospy.Publisher('/bebop/reset', Empty, queue_size=1)
    rospy.init_node('kill_button', anonymous=True)

    while not rospy.is_shutdown():
        
        if os.path.exists(killing_file_path):
            print("killing")
            pub.publish()
        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

