from std_msgs.msg import String, Empty, Bool

from bebop_msgs.msg import Ardrone3PilotingStateAlertStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

from geometry_msgs.msg import Twist, Point, Pose
import roslib
import os
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool, UInt8

from cv_bridge import CvBridge, CvBridgeError

pub_state = rospy.Publisher("/state_machine/state",
                            String, queue_size=1)

import rospkg
import json
import ros_numpy
import numpy as np
rospack = rospkg.RosPack()
positions_path = str(rospack.get_path('drone_control')+'/config/recorded_routines.json')

with open(positions_path, 'r') as json_data_file:
    moving_routines = json.load(json_data_file)
    

class takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'error'])    
        self.takeoff_topic = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
        self.camera_angle_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_runnig_state", Bool, queue_size=1)
        self.angle_msg = Twist()
        self.angle_msg.angular.y = 3
        self.z = 0
    def altitude_callback(self, data):
        self.z = data.altitude
        print(data.altitude)

    def execute(self, userdata):
        altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",Ardrone3PilotingStateAltitudeChanged, self.altitude_callback)
        rospy.loginfo('Executing state takeoff')
        for i in range(10):
            self.camera_angle_pub.publish(self.angle_msg)
            self.running_aligned_pub.publish(False)
            rospy.sleep(0.05)
        rospy.loginfo('Camera Alined')

        for i in range(10):
            self.takeoff_topic.publish(Empty())
            rospy.sleep(0.1)
        while self.z < 0.8 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        altitude_sub.unregister()
        return 'done'


class land_now(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.land_topic = rospy.Publisher("/bebop/land", Empty,queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('land')

        for i in range(200):
            self.land_topic.publish(Empty())
            rospy.sleep(0.05)
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

        self.running_rect_pub= rospy.Publisher("/cv_detection/rectangle_detector/set_runnig_state", Bool,queue_size=1)
        # self.running_pub= rospy.Publisher("cv_detection/color_range/set_runnig_state", Bool, queue_size=1)
        # self.color_pub= rospy.Publisher("cv_detection/color_range/set_color", Bool)
        self.goal_point_pub = rospy.Publisher("/control/align_reference/set_goal_point", Point, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_runnig_state", Bool, queue_size=1)
        
    #     self.aligned_sub = rospy.Subscriber("/control/align_reference/aligned", Bool, self.aligned_callback, queue_size=1)

    # def aligned_callback()
    def execute(self, userdata):
        self.running_aligned_pub.publish(True)
        self.running_rect_pub.publish(True)
        # self.color_pub.publish("green")
        self.running_aligned_pub.publish(True)
        p = Point()
        p.x = 856/2
        p.y = 480/2
        p.z = 135
        self.goal_point_pub.publish(p)
        print("ok")


        rospy.wait_for_message("/control/align_reference/aligned", Bool)
        # rospy.sleep(1)
        # print(done)
        self.running_rect_pub.publish(False)
        self.running_aligned_pub.publish(False)
        

        return 'flag_aligned'

class change_view_flag (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):

        return 'done'

class capture_flag (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        camera_topic = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
        camera_init_angle = Twist()
        camera_init_angle.angular.y = 3 # looking front
        camera_topic.publish(camera_init_angle)

        self.running_rect_pub= rospy.Publisher("/cv_detection/rectangle_detector/set_runnig_state", Bool,queue_size=1)
        self.save_detection_pub = rospy.Publisher("cv_detection/rectangle_detector/save_detection", String, queue_size=1)

        self.goal_point_pub = rospy.Publisher("/control/align_reference/set_goal_point", Point, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_runnig_state", Bool, queue_size=1)
        self.bridge = CvBridge()

        

    def execute(self, userdata):
        self.running_rect_pub.publish(True)

        count_erros = 0
        detection_saved = False
        while not detection_saved:
            self.save_detection_pub.publish('flag.png')
            try:
                detection_saved = rospy.wait_for_message('cv_detection/rectangle_detector/detection_saved', Bool, 0.5)
            except:
                count_erros += 1
                print(count_erros)
        self.running_rect_pub.publish(False)
        return 'done'



class face_shelf (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.pose_pub = rospy.Publisher("/control/position", Pose, queue_size=1)
        self.running_control_pub = rospy.Publisher("/control/set_runnig_state", Bool, queue_size=1)
        # self.running_control_pub = rospy.Publisher("/control/aligned", Bool, queue_size=1)


    def execute(self, userdata):

        self.running_control_pub.publish(True)
        # self.pose_pub.publish()
        routine_name = 'rosana'

        if routine_name in moving_routines:
            for position in moving_routines[routine_name]:
                print("------------- pose ----------")
                new_pose = ros_numpy.msgify(Pose,np.array(position))
                print(new_pose)
                self.pose_pub.publish(new_pose)
                rospy.wait_for_message("/control/aligned", Bool)
        else:
            print("no routine named: "+routine_name)
            print(moving_routines)
        self.running_control_pub.publish(False)
        return 'done'

class align_window(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        camera_topic = rospy.Publisher("/bebop/camera_control", Twist, queue_size=1)
        camera_init_angle = Twist()
        camera_init_angle.angular.y = 3 # looking front
        camera_topic.publish(camera_init_angle)

        # self.running_rect_pub= rospy.Publisher("/cv_detection/rectangle_detector/set_runnig_state", Bool,queue_size=1)
        self.running_color_pub= rospy.Publisher("cv_detection/color_range/set_runnig_state", Bool, queue_size=1)
        self.color_pub= rospy.Publisher("cv_detection/color_range/set_color", String,queue_size=1)
        self.goal_point_pub = rospy.Publisher("/control/align_reference/set_goal_point", Point, queue_size=1)
        self.running_aligned_pub = rospy.Publisher("/control/align_reference/set_runnig_state", Bool, queue_size=1)
        
    #     self.aligned_sub = rospy.Subscriber("/control/align_reference/aligned", Bool, self.aligned_callback, queue_size=1)

    # def aligned_callback()
    def execute(self, userdata):
        self.running_aligned_pub.publish(True)
        self.running_color_pub.publish(True)
        self.color_pub.publish("green_portal")
        self.running_aligned_pub.publish(True)
        p = Point()
        p.x = 856/2
        p.y = 480/2
        p.z = 300
        self.goal_point_pub.publish(p)
        print("ok")


        rospy.wait_for_message("/control/align_reference/aligned", Bool)
        # rospy.sleep(1)
        # print(done)
        self.running_color_pub.publish(False)
        self.running_aligned_pub.publish(False)
        

        return 'done'

class pass_through_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.pose_pub = rospy.Publisher("/control/position", Pose, queue_size=1)
        self.relative_pose_pub = rospy.Publisher("/control/position_relative", Pose, queue_size=1)
        self.running_control_pub = rospy.Publisher("/control/set_runnig_state", Bool, queue_size=1)
        # self.running_control_pub = rospy.Publisher("/control/aligned", Bool, queue_size=1)


    def execute(self, userdata):
        rospy.sleep(2)
        current_pose = rospy.wait_for_message("control/current_position", Pose)
        current_pose.position.z+=0.05
        self.pose_pub.publish(current_pose) #sticks to current position
        self.running_control_pub.publish(True)
        rospy.wait_for_message("/control/aligned", Bool)
        p = Pose()
        p.position.x = 1
        p.orientation.w = 1
        self.relative_pose_pub.publish(p)
        rospy.wait_for_message("/control/aligned", Bool)
        rospy.sleep(2)
        self.running_control_pub.publish(False)
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

class drop_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.move_topic = rospy.Publisher(
            "/bebop/cmd_vel", Twist, queue_size=1)
        self.flip_pub = rospy.Publisher(
            "/bebop/flip", UInt8, queue_size=1)

    def control(self, x, y, z, t):
        twist = Twist()
        speed = 4
        twist.linear.x = x * speed
        twist.linear.y = y * speed
        twist.linear.z = z * speed
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        for i in range(int(t*10)):
            self.move_topic.publish(twist)
            rospy.sleep(0.1)

    def execute(self, userdata):
        rospy.loginfo('drop_box')
        self.flip_pub.publish(1)
        # self.control(0, 0, -1, 0.3)
        # self.control(-0, 0, 100,1)
        # self.control(-1, 0, 0,0.3)

        rospy.sleep(1)
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