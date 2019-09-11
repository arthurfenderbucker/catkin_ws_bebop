from std_msgs.msg import String, Empty, Bool

from bebop_msgs.msg import Ardrone3PilotingStateAlertStateChanged

from geometry_msgs.msg import Twist
import roslib
import os
import rospy
import smach
import smach_ros

pub_state = rospy.Publisher("/state_machine/state",
                            String, queue_size=1)


class takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])    
        self.takeoff_topic = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state takeoff')

        for i in range(10):
            self.takeoff_topic.publish(Empty())
            rospy.sleep(0.1)
        rospy.sleep(6)
        # rospy.loginfo(state_msg.state)
        # try:
        #     rospy.wait_for_message("/bebop/states/ardPilotingState/FlyingStateChanged",Ardrone3PilotingStateAlertStateChanged,15.0)
        #     rospy.wait_for_message("/bebop/states/ardPilotingState/FlyingStateChanged",Ardrone3PilotingStateAlertStateChanged,15.0)
        #     return 'takeoff_done'
        # except Exception as e:
        #     print(e)
        #     return 'error'
        return 'done'


class land_now(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.land_topic = rospy.Publisher("/bebop/land", Empty, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('land')

        for i in range(200):
            self.land_topic.publish(Empty())
        rospy.sleep(1)
        # self.condition = ""
        return 'done'


# ======================== Flag ================================
class align_flag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flag_aligned','reference_lost', 'too_many_attempts'])
        camera_topic = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
        camera_init_angle = Twist()
        camera_init_angle.angular.y = 3 # looking front
        camera_topic.publish(camera_init_angle)
        
    def execute(self, userdata):

        rospy.sleep(1)
        return 'flag_aligned'

class change_view_flag (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'

class face_shelf (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'


# ================================= QR Code ===================================
class align_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shelf_aligned','reference_lost', 'too_many_attempts'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'

class change_view_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'

class face_boxes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'


# ====================== QR code =================
class zigzag_qr_code(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','reference_lost'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'


class change_view_qr_code(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        rospy.sleep(1)
        return 'done'


class align_box(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['done', 'erro'])
        self.c = color_detection("red")
    def execute(self, userdata):


        return 'done'
class pickup_box(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['done', 'erro'])
        rospy.Subscriber("/state_machine/follow/find_condition",
                         String, self.callback)
        self.condition = ""

    def callback(self, data):
        print(data)
        self.condition = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state find window')
        pub_state.publish("follow")
        while self.condition != "ok" and not rospy.is_shutdown():
            pass
        self.condition = ""
        return 'done'



class square(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'erro'])

        self.move_topic = rospy.Publisher(
            "/bebop/cmd_vel", Twist, queue_size=1)

    def control(self, x, y, z):
        twist = Twist()
        speed = 4
        twist.linear.x = x * speed
        twist.linear.y = y * speed
        twist.linear.z = z * speed
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        for i in range(20):
            self.move_topic.publish(twist)
            rospy.sleep(0.01)

    def execute(self, userdata):
        rospy.loginfo('Square')
        self.control(1, 0, 1)
        rospy.sleep(6)
        self.control(2, 0, 0)
        rospy.sleep(3)
        self.control(1, 1, 0)
        rospy.sleep(3)
        self.control(-1, 0, 0)
        rospy.sleep(3)
        self.control(1, -1, 0)
        rospy.sleep(3)
        self.control(1, 0, 0)
        self.condition = ""
        return 'done'


# define state find_window

# class follow(smach.State):
#     def __init__(self):
#         smach.State.__init__(
#             self, outcomes=['done', 'erro'])
#         rospy.Subscriber("/state_machine/follow/find_condition",
#                          String, self.callback)
#         self.condition = ""

#     def callback(self, data):
#         print(data)
#         self.condition = data.data

#     def execute(self, userdata):
#         rospy.loginfo('Executing state find window')
#         pub_state.publish("follow")
#         while self.condition != "ok" and not rospy.is_shutdown():
#             pass
#         self.condition = ""
#         return 'done'


# class find_window(smach.State):
#     def __init__(self):
#         smach.State.__init__(
#             self, outcomes=['window found', 'not on view', 'too many attempts'])
#         rospy.Subscriber("/state_machine/window/find_condition",
#                          String, self.callback)
#         self.condition = ""

#     def callback(self, data):
#         print(data)
#         self.condition = data.data

#     def execute(self, userdata):
#         rospy.loginfo('Executing state find window')
#         pub_state.publish("find_window")
#         while self.condition != "ok" and not rospy.is_shutdown():
#             pass
#         self.condition = ""
#         return 'window found'

# # define state change_view


# class change_view(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['view_changed'])

#     def execute(self, userdata):
#         rospy.loginfo('Executing state find window')

#         return 'view_changed'

# # define state through_window_state


# class through_window_state(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['completed window'])
#         rospy.Subscriber("/state_machine/window/through_condition",
#                          String, self.callback)
#         self.condition = ""

#     def callback(self, data):
#         print(data)
#         self.condition = data.data

#     def execute(self, userdata):
#         rospy.loginfo('Executing state through window')
#         pub_state.publish("through_window")
#         while self.condition != "ok" and not rospy.is_shutdown():
#             pass
#         self.condition = ""
#         # pub_state.publish("")
#         return 'completed window'

# # define state rope


# class find_rope(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['rope_found'])
#         rospy.Subscriber("/state_machine/rope/find_condition",
#                          String, self.callback)
#         self.condition = ""

#     def callback(self, data):
#         print(data)
#         self.condition = data.data

#     def execute(self, userdata):
#         rospy.loginfo('Executing state find_rope')
#         pub_state.publish("find_rope")
#         while self.condition != "ok" and not rospy.is_shutdown():
#             pass
#         self.condition = ""
#         rospy.loginfo("exited find_rope")
#         return 'rope_found'