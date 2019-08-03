#!/usr/bin/env python

import rospy
import smach
import pandas as pd
import os
import time
import smach_ros
from QrCode import QrCode
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from flag import cutflag
from alphanumeric import DetectAlphanumeric

velocity_message = Twist()

rospy.init_node("Inventory_State_Machine")
rate = rospy.Rate(20)
from set_position import set_x_relative_position, set_y_relative_position, set_z_relative_position

bebop_image =  np.zeros((256, 256, 1), dtype = "uint8")
flag_image =  np.zeros((256, 256, 1), dtype = "uint8")

def image_callback(image):
    global bebop_image
    bebop_image = bridge.imgmsg_to_cv2(image, "bgr8")

velocity_position_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
image_sub = rospy.Subscriber("/bebop/image_raw", Image, image_callback, queue_size=2)

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
        self.count = 0
    def execute(self):
        global flag_image
        if self.count >= 10:
            return 'failsafe'
        try:
            flag_image = cutflag(bebop_image)
            return 'done'
        except Exception as e:
            self.count += 1
            print(e)
            return 'fail'


class Inventory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.file = open("inventory.txt", "a")
        self.alph_dec = DetectAlphanumeric()
        self.qr_dec = QrCode()



        self.s = 1 # Variavel para indicar se e a segunda vez que o inventario e' realizado
        self.COLUMNS = 4
        self.ROWS = 3
        self.COLUMN_TO_COLUMN = 0.8
        self.ROW_TO_ROW = 0.6

        self.qrs = pd.DataFrame(index=range(0,ROWS-1), columns=range(0,COLUMNS-1))
        self.poses = pd.DataFrame(index=range(0,ROWS-1), columns=range(0,COLUMNS-1))

    def read_alph(self):
        rospy.loginfo('Reading Alphanumeric Digits!')
        try:
            text = self.alph_dec.detect()
        except Exception as e:
            rospy.logerr("Error detecting alphanumerics!")
            rospy.logerr(e)

        return text

    def read_qr(self):
        rospy.loginfo('Reading QR Codes!')
        try:
            data = self.qr_dec.detect()
        except Exception as e:
            rospy.logerr("Error detecting QR Codes!")
            rospy.logerr(e)

        return data

    def execute(self, userdata):
        rospy.loginfo('Executing state Inventory')
        rospy.sleep(2)

        shelf_id = self.read_alph()
        rospy.loginfo('Shelf Identification Code: ' + str(shelf_id))

        right = 1 # Variable indicating if the drone goes right (oscilates between 1 and -1)

        for row in range(ROWS):
            rospy.loginfo('Row: ' + str(row))
            self.read_qr() # Isso e necessario?
            for column in range(COLUMNS):
                rospy.loginfo('Column: ' + str(column))
                qrs[row][column] = self.read_qr()
                rospy.loginfo('QR Code: ' + str(qr[row][column]))

                poses[row][column] = self.read_alph()
                rospy.loginfo('Alphanumeric: ' + str(qr[row][column]))

                set_x_relative_position(right*COLUMN_TO_COLUMN)

            set_z_relative_position(ROW_TO_ROW)
            right = -right
        pd.concat([qrs, poses], axis=0).to_csv('inventory' + str(s) + '.csv')
        self.s += 1
        if s >= 2:
            return 'second'
        else:
            return 'succeeded'


class CrossShelves(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
    def execute(self):
        ##################
        #   Implementar  #
        ##################
        return 'succeeded'

class OverflyShelves(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self):
        ##################
        #   Implementar  #
        ##################
        return 'done'

class PackPick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self):
        ##################
        #   Implementar  #
        ##################
        return 'done'

class PackRelese(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self):
        ##################
        #   Implementar  #
        ##################
        return 'done'

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
        ####### Transition State Necessary

        smach.StateMachine.add('INVENTORY', Inventory(),
                                transitions={'succeeded':'CROSS_SHELVES', 'aborted':'CROSS_SHELVES', 'second':'PACK_PICK'})

        smach.StateMachine.add('CROSS_SHELVES', CrossShelves(),
                                transitions={'succeeded':'LAND', 'aborted':'OVERFLY_SHELVES'})

        smach.StateMachine.add('OVERFLY_SHELVES', OverflyShelves(),
                                transitions={'done':'INVENTORY'})

        smach.StateMachine.add('PACK_PICK', PackPick(),
                                transitions={'done':'PACK_RELEASE'})

        smach.StateMachine.add('PACK_RELEASE', PackRelese(),
                                transitions={'done':'LAND'})

        smach.StateMachine.add('LAND', Land(),
                                transitions={'done':'Mission executed successfully!'})


     # Execute SMACH plan
    outcome = sm.execute()

if __name__ == "__main__":
    main()
