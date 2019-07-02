#!/usr/bin/env python

import rospy
import smach
import os
import smach_ros
from QrCode import QrCode
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from alphanumeric import DetectAlphanumeric

velocity_message = Twist()

rospy.init_node("Inventory_State_Machine")
from set_position import set_x_relative_position
rate = rospy.Rate(20)

velocity_position_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)


def set_w(theta):
    theta = (theta*3.14159265)/180
    tau = 1.93
    t = theta/tau
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 1
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    last_time = rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() - last_time < t:
        velocity_position_pub.publish(velocity_message)
        rate.sleep()

def set_velocity(x, y, z, t):
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0
    velocity_message.linear.x = x
    velocity_message.linear.y = y
    velocity_message.linear.z = z

    rospy.loginfo('Velocity: (' + str(velocity_message.linear.x) + ', ' + str(velocity_message.linear.y) + ', ' + str(velocity_message.linear.z) + ')')

    last_time = rospy.get_rostime().to_sec()

    while rospy.get_rostime().to_sec() - last_time < t:
        velocity_position_pub.publish(velocity_message)
        rate.sleep()

    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_position_pub.publish(velocity_message)


############################### Smach States ###################################
class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','aborted'])
        self.pub = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Takeoff')
        for i in range(20):
            self.pub.publish(Empty())
            rate.sleep()
        rospy.sleep(10)
        return 'done'


class Flag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','fail', 'failsafe'])
    def execute(self):
        pass

class Inventory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.file = open("inventory.txt", "a")
        self.alph_dec = DetectAlphanumeric()
        self.qr_dec = QrCode()
        #os.system("gnome-terminal -x python alphanumeric.py")

    def execute(self, userdata):
        rospy.loginfo('Executing state Inventory')
        rospy.loginfo('Spinning')
        set_w(90)
        rospy.sleep(5)
        set_velocity(0.2, 0, 0, 1)
        rospy.loginfo('Reading Alphanumeric Digits!')
        text = self.alph_dec.detect()
        print(text)
        self.file.write(str(text) + '\n\n')

        rospy.sleep(4)
        set_velocity(0, 0.1, 0, 1.7)
        rospy.loginfo('Reading Alphanumeric Digits!')
        text = self.alph_dec.detect()
        print(text)
        self.file.write(str(text) + '\n\n')
        rospy.sleep(4)
        set_velocity(0, 0.1, 0, 1.7)
        rospy.loginfo('Reading Alphanumeric Digits!')
        text = self.alph_dec.detect()
        print(text)
        self.file.write(str(text) + '\n\n')
        # rospy.loginfo('Reading QR Codes!')
        # data = self.qr_dec.detect()
        # print(data)
        last_time = rospy.get_rostime().to_sec()
        # rospy.loginfo('Reading QR Codes!')
        # while rospy.get_rostime().to_sec() - last_time < 10:
        #     data = self.qr_dec.detect()
        #     print(data)
        #     rate.sleep()
        #     self.file.write(str(data) + '\n')
        # text = self.alph_dec.detect()
        # while rospy.get_rostime().to_sec() - last_time < 10:
        #     text = self.alph_dec.detect()
        #     print(text)
        # self.file.write(str(text) + '\n\n')
        #set_velocity(0, 0.2, 0)
        rospy.sleep(5)
        last_time = rospy.get_rostime().to_sec()
        # rospy.loginfo('Reading QR Codes!')
        # while rospy.get_rostime().to_sec() - last_time < 10:
        #     data = self.qr_dec.detect()
        #     print(data)
        # self.file.write(str(data) + '\n')
        rospy.loginfo('Reading Alphanumeric Digits!')
        # while rospy.get_rostime().to_sec() - last_time < 10:
        #     text = self.alph_dec.detect()
        #     print(text)
        # self.file.write(str(text) + '\n\n')
        return 'succeeded'

class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Land')
        self.pub.publish(Empty())
        return 'done'
################################################################################

def main():
    rospy.logwarn("Running State Machine")
    sm = smach.StateMachine(outcomes=['Mission executed successfully!'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TAKEOFF', Takeoff(),
                                transitions={'done':'INVENTORY', 'aborted':'Mission executed successfully!'})

        smach.StateMachine.add('FLAG', Flag(),
                                transitions={'done':'INVENTORY', 'fail':'FLAG', 'failsafe':'INVENTORY'})

        smach.StateMachine.add('INVENTORY', Inventory(),
                                transitions={'succeeded':'LAND', 'aborted':'Mission executed successfully!'})

        smach.StateMachine.add('LAND', Land(),
                                transitions={'done':'Mission executed successfully!'})

     # Execute SMACH plan
    outcome = sm.execute()

if __name__ == "__main__":
    main()
