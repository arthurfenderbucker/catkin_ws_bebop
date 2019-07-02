#!/usr/bin/env python

import rospy
import smach
import smach_ros
from QrCode import QrCode
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from alphanumeric import DetectAlphanumeric

velocity_message = Twist()

rospy.init_node("Inventory_State_Machine")
rate = rospy.Rate(20)
print("foi\n")
from set_position import set_x_relative_position

velocity_position_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
def spinalittle():
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0.4
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    last_time = rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() - last_time < 4:
        velocity_position_pub.publish(velocity_message)
        rate.sleep()

def walkalittle():
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0
    velocity_message.linear.x = 1
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    rospy.loginfo('Velocity: (' + str(velocity_message.linear.x) + ', ' + str(velocity_message.linear.y) + ', ' + str(velocity_message.linear.z) + ')')
    last_time = rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() - last_time < 9:
        velocity_position_pub.publish(velocity_message)
        rate.sleep()

class Takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','aborted'])
        self.pub = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Takeoff')
        for i in range(20):
            self.pub.publish(Empty())
            rate.sleep()
        rospy.sleep(8)
        rospy.loginfo('Spinning')
        #spinalittle()
        rospy.sleep(5)
        walkalittle()
        return 'done'

class Inventory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.file = open("inventory.txt", "a")

    def execute(self, userdata):
        rospy.loginfo('Executing state Inventory')
        alph_dec = DetectAlphanumeric()
        qr_dec = QrCode()
        last_time = rospy.get_rostime().to_sec()
        rospy.loginfo('Reading QR Codes!')
        while rospy.get_rostime().to_sec() - last_time < 15:
            data = qr_dec.detect()
            print(data)
        rospy.loginfo('Reading Alphanumeric Digits!')
        text = alph_dec.detect()
        while rospy.get_rostime().to_sec() - last_time < 15:
            text = alph_dec.detect()
            print(text)
        self.file.write(str(text) + '\n')
        return 'succeeded'

class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Land')
        self.pub.publish(Empty())
        return 'done'


def main():
    rospy.logwarn("Running State Machine")
    sm = smach.StateMachine(outcomes=['Mission executed successfully!'])
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TAKEOFF', Takeoff(),
                                transitions={'done':'INVENTORY', 'aborted':'Mission executed successfully!'})

        smach.StateMachine.add('INVENTORY', Inventory(),
                                transitions={'succeeded':'LAND', 'aborted':'Mission executed successfully!'})

        smach.StateMachine.add('LAND', Land(),
                                transitions={'done':'Mission executed successfully!'})

     # Execute SMACH plan
    outcome = sm.execute()

if __name__ == "__main__":
    main()
