#!/usr/bin/env python
import rospy
import numpy as np
import math
# import mavros_msgs

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
# from mavros_msgs import srv
# from mavros_msgs.msg import State
from dronecontrol.msg import Vector3D

#=================Parameter Initializaiton========================,
goal_pose = PoseStamped()
current_pose = PoseStamped()
set_velocity = TwistStamped()
# current_state = State()
current_state = None
control_mode = "position"


def altitude_hold():
    global goal_pose
    goal_pose.pose.position.z = 3
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.x = 0


def position_callback(goal_vec):
    global goal_pose
    goal_pose.pose.position.z = goal_vec.z
    goal_pose.pose.position.y = goal_vec.y
    goal_pose.pose.position.x = goal_vec.x

    global control_mode
    control_mode = "position"
    rospy.loginfo("got position goal")
    rospy.loginfo("x: " + str(goal_vec.x) + " y: " +
                  str(goal_vec.y) + " z: " + str(goal_vec.z))


def velocity_callback(goal_vec):
    global set_velocity
    set_velocity.twist.linear.z = goal_vec.z
    set_velocity.twist.linear.y = goal_vec.y
    set_velocity.twist.linear.x = goal_vec.x

    global control_mode
    control_mode = "velocity"
    rospy.loginfo("got velocity goal")
    rospy.loginfo("x: " + str(goal_vec.x) + " y: " +
                  str(goal_vec.y) + " z: " + str(goal_vec.z))

#==============Call Back Functions=====================


def pos_sub_callback(pose_sub_data):
    global current_pose
    current_pose = pose_sub_data


def state_callback(state_data):
    global current_state
    current_state = state_data


#============Intialize Node, Publishers and Subscribers=================
rospy.init_node('Vel_Control_Node', anonymous=True)
rate = rospy.Rate(20)  # publish at 20 Hz
#                           mavros
# local_position_subscribe = rospy.Subscriber(
#     '/mavros/local_position/pose', PoseStamped, pos_sub_callback)
# local_position_pub = rospy.Publisher(
#     '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
# setpoint_velocity_pub = rospy.Publisher(
#     '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
# state_status_subscribe = rospy.Subscriber(
#     '/mavros/state', State, state_callback)

#                           bebop
setpoint_velocity_pub = rospy.Publisher(
    '/bebop/cmd_vel', Twist, queue_size=10)

set_position = rospy.Subscriber(
    '/controle/position', Vector3D, position_callback)
set_position = rospy.Subscriber(
    '/controle/velocity', Vector3D, velocity_callback)
rospy.loginfo("setup ok")

altitude_hold()


#============Define Velocity==============================================
# set_velocity.twist.linear.x = 1  # moving 1m/s at x direction

# arm = rospy.ServiceProxy('/mavros/cmd/arming',
#                          mavros_msgs.srv.CommandBool)
# set_mode = rospy.ServiceProxy(
#     '/mavros/set_mode', mavros_msgs.srv.SetMode)

# for i in range(300):  # arm and takeoff
    # local_position_pub.publish(goal_pose)
    # rate.sleep()

while not rospy.is_shutdown():

    # if control_mode == "position":
    #     local_position_pub.publish(goal_pose)

    # if current_state.mode != "OFFBOARD" or not current_state.armed:

    #     arm(True)

    #     mode = set_mode(custom_mode='OFFBOARD')

    #     # if mode.success:
    #     if current_state.mode == "OFFBOARD":
    #         rospy.loginfo('Switched to OFFBOARD mode!')

    if control_mode == "velocity":
        setpoint_velocity_pub.publish(set_velocity)

    rate.sleep()
