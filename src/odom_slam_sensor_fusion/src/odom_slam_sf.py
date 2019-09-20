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

        # self.current_coord_pub = rospy.Publisher(
        #     "odom_slam_sf/current_position", Point, queue_size=1)

        rospy.loginfo("setup ok")

    # ------------ topics callbacks -----------
    def odometry_callback(self, odom):

        # self.angle_pose = odom.pose.pose.orientation.z

        # if rospy.get_time() - self.last_vso_time > 0.2:
        #     self.trust_vso = 0
        # if not self.trust_vso:
        #     rospy.loginfo("NO FEATURES!!! using bebop odom")

        print(ros_numpy.numpify(odom.pose.pose))
        self.odom_pose_raw = ros_numpy.numpify(odom.pose.pose)
        
        # odom_coord_raw = ros_numpy.numpify(odom.pose.pose)[:3,3]

        
        # odom_coord_raw = np.hstack((odom_coord_raw,[1]))
        # self.odom_coord = np.dot(odom_coord_raw, self.odom_to_slam_tf.T)
        self.current_coord = self.odom_coord
        # p = Pose()
        # p.position.x = self.current_coord[0]
        # p.position.y = self.current_coord[1]
        # p.position.z = self.current_coord[2]

        

    def slam_callback(self,pose):
        self.slam_pose_raw = ros_numpy.numpify(pose.pose) #homogeneous transformation matrix from the origin
        
        #adjust the received raw_slam_coord to the desire coords system
        # self.slam_coord = np.dot(np.hstack((raw_slam_coord,[1])),self.slam_tf_matrix)
        self.slam_pose = np.dot(self.slam_pose_correction, self.slam_pose_raw)
        
        if not self.odom_pose_raw == None:
            #calculates the transfor matrix for the odom position to the slam coords system
            self.odom_pose_correction = np.dot(self.slam_pose, np.linalg.inv(self.odom_pose_raw))
            print(self.slam_pose)
            print()
            self.current_pose = self.slam_pose
            # self.odom_pose_correction = self.calculte_odom_to_slam_tf(odom_coord_raw, self.slam_coord)
        else:
            rospy.loginfo("Havent received any slam coord yet!")
        # self.current_coord = self.slam_coord
        # rospy.loginfo("POSE: " + str(self.current_coord))

    # ----------------------Sensor Fusion functions--------------------------
    def calculte_odom_to_slam_tf(self, odom_tf, slam_tf ):
        odom_tf_inv = np.linalg.inv(odom_tf)
        return np.dot(self.slam_tf, odom_tf_inv)
        # """
        # odom_coord = np.array([x,y,z])  slam_coord = np.array([x,y,z])
        # dinamicaly calculates the rotation angle and offset position of 2 given coord considering that
        # both coords system have the same scale and and are properly oriented with z axis pointing up:
        # returns the odom_to_slam_tf (3x4 matrix)
        # np.dot( odom_coord, odom_to_slam_tf.T) = slam_coord"""
        #assumes other known possitions given the considerations
    
        # x = np.vstack((odom_coord,odom_coord,odom_coord,odom_coord))+np.eye(4,3 )
        # y = np.vstack((slam_coord,slam_coord,slam_coord,slam_coord))+np.eye(4,3 )
        # odom_coord_expanded = np.hstack([x,np.ones((len(x),1))])
        # # print(odom_coord_expanded)
        # #yeah... this is kind of a laizy approach...
        # reg = LinearRegression().fit(x, y)
        # return np.hstack([reg.coef_,np.expand_dims(reg.intercept_,axis=1)]) #3x4 


    def run(self):
        while not rospy.is_shutdown():

            p = ros_numpy(self.current_pose, Pose)
            self.current_pose.publish(p)

            self.rate.sleep()


if __name__ == "__main__":
    c = odom_slam_sf()
    c.run()
