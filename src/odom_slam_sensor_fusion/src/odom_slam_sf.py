#!/usr/bin/env python
import rospy
import numpy as np
import math
from sklearn.linear_model import LinearRegression

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
import ros_numpy


from drone_control.cfg import ControlConfig
import rospkg
import json

import time


rospack = rospkg.RosPack()
class odom_slam_sf(object): # visual odometry drone controler
    
    current_coord = np.array([0.0,0.0,0.0])
    angle_pose = 0.0
    odom_to_slam_tf = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]]) # initially no transformation
    odom_pose = np.array([0.0,0.0,0.0])
    slam_coord = None

    def __init__(self):

        #setup node
        rospy.init_node('Odom_slam_sf', anonymous=True)
        self.rate = rospy.Rate(60)  # refresh rate (Hz)

        self.odom_topic = rospy.get_param('~odom_topic','/bebop/odom/')
        self.slam_pose_topic = rospy.get_param('~slam_pose_topic','/orb_slam2_mono/pose')
        config_path = rospy.get_param('~config_path',str(rospack.get_path('odom_slam_sensor_fusion')+'/config/maps/recorded_maps.json'))
        map_name = rospy.get_param('~map_name',"test_map")

       
        try:
            with open(config_path, 'r') as json_data_file:
                calibration_file_data = json.load(json_data_file)
        except:
            raise Exception("error openning the config file: "+config_path)

        if map_name in calibration_file_data.keys():
            calibration_data = calibration_file_data[map_name]
            
            self.slam_pose_correction = np.array(calibration_data["pose_correction_matrix"])
            rospy.loginfo("calibration loaded")
        else:
            rospy.logerr("error loading \""+map_name+"\" calibration data")
            raise Exception( "please run calibrate_slam.py node")


        rospy.Subscriber( self.slam_pose_topic , PoseStamped, self.slam_callback)

        # rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged', Ardrone3PilotingStateAltitudeChanged, self.altitude_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback)

        self.current_coord_pub = rospy.Publisher(
            "odom_slam_sf/current_coord", Point, queue_size=1)

        self.current_pose_pub = rospy.Publisher(
            "odom_slam_sf/current_position", Pose, queue_size=1)

        rospy.loginfo("setup ok")

    # ------------ topics callbacks -----------
    def odometry_callback(self, odom):

        print(ros_numpy.numpify(odom.pose.pose))
        self.odom_pose_raw = ros_numpy.numpify(odom.pose.pose)
        
        self.current_coord = self.odom_coord

        

    def slam_callback(self,pose):
        self.slam_pose_raw = ros_numpy.numpify(pose.pose) #homogeneous transformation matrix from the origin
        
        #adjust the received raw_slam_coord to the desire coords system
        # self.slam_coord = np.dot(np.hstack((raw_slam_coord,[1])),self.slam_tf_matrix)
        self.slam_pose = np.dot(self.slam_pose_correction, self.slam_pose_raw)
        
        if not self.odom_pose_raw == None:
            #calculates the transfor matrix for the odom position to the modified slam coords system (assumed as true value)
            self.odom_pose_correction = np.dot(np.linalg.inv(self.odom_pose_raw),self.slam_pose)
            self.current_pose = self.slam_pose
        else:
            rospy.loginfo("Havent received any slam coord yet!")

    # ----------------------Sensor Fusion functions--------------------------



    def run(self):
        while not rospy.is_shutdown():

            p = ros_numpy(Pose, self.current_pose)
            self.current_pose.publish(p)

            self.rate.sleep()


if __name__ == "__main__":
    c = odom_slam_sf()
    c.run()
