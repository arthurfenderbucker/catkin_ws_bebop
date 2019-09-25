#!/usr/bin/env python
import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
import ros_numpy

from PID import PID

from dynamic_reconfigure.server import Server
from drone_control.cfg import ControlConfig

import time
import tf


class vso_controler(object): # visual odometry drone controler
    
    goal_pose = Pose()
    current_pose = Pose()
    goal_pose.orientation.w = 1
    goal_pose.position.z = 1
    positioning_vel = np.array([0.0,0.0,0.0,0.0])


    pid_x = PID(P=0.05,I=0.0000,D=0.035)
    pid_y = PID(P=0.05,I=0.0000,D=0.035)
    pid_z = PID(P=0.18,I=0.00001,D=0.0012)
    pid_ang = PID(P=0.068,I=0.0,D=0.003)

    camera_angle = Twist()
    setted_vel = Twist()

    control_mode = "position" # position or velocity  
    precision = np.array([0.15,0.15,0.1,0.1])
    count_aligned = 0
    def __init__(self):

        #setup node
        rospy.init_node('Vel_Control_Node', anonymous=True)
        self.rate = rospy.Rate(60)  # refresh rate (Hz)

        #topics and services
        self.setpoint_velocity_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.camera_angle_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)

        self.running = rospy.get_param('~running',True)
        self.vso_on = rospy.get_param('~vso_on',True)
        self.config_file = rospy.get_param('~config_file',"default.json")

        calibrate_pid = rospy.get_param('~calibrate_pid',False)

        rospy.Subscriber('/bebop/land', Empty, self.land)
        rospy.Subscriber('/bebop/reset', Empty, self.land)
        # rospy.Subscriber('/bebop/takeoff', Empty, self.takeoff)
        
        rospy.Subscriber('/odom_slam_sf/current_pose', Pose, self.current_pose_callback)

        
        rospy.Subscriber('/control/position', Pose, self.position_callback)
        rospy.Subscriber('/control/position_relative', Pose, self.position_relative_callback)
        rospy.Subscriber('/control/velocity', Point, self.velocity_callback, queue_size=10)
        rospy.Subscriber('/control/set_precision', Point, self.set_precision)
        rospy.Subscriber('/control/pickup_box', Empty, self.pickup_box)

        # rospy.Subscriber('/control/land', Empty, self.land)

        rospy.Service('/control/calibrate_pid', SetBool, self.set_calibrate_pid)

        self.running_sub = rospy.Subscriber(
            "control/set_runnig_state", Bool, self.set_runninng_state, queue_size=1)
        self.current_pose_pub = rospy.Publisher(
            "control/current_position", Pose, queue_size=1)
        self.aligned = rospy.Publisher(
            "/control/aligned", Bool, queue_size=1)
            
        #dynamic parameters serve
        if calibrate_pid:
            srv = Server(ControlConfig, self.parameters_callback)
        
        t = time.time()
        rospy.loginfo("aligning camera")
        while time.time() - t < 1:
            self.align_camera()
        rospy.loginfo("setup ok")
        self.pid_setpoint(self.goal_pose)
        # self.reset()

    # ------------ topics callbacks -----------
    def set_precision(self,data):
        self.precision = np.array([data.x,data.y,data.z,self.precision[3]])
    def set_runninng_state(self,boolean_state):
        self.running = boolean_state.data
        self.reset_pid()

    def land(self,callback_data):
        self.running = False

    def takeoff(self,callback_data):
        self.align_camera()
        
    def position_callback(self, goal_pose):
        self.goal_pose = goal_pose
        self.goal_pose_np = ros_numpy.numpify(goal_pose)
        self.goal_z_ang = self.euler_from_pose(self.goal_pose)[2]
        self.pid_setpoint(self.goal_pose)
        self.control_mode = "position"
        self.count_aligned = 0

    def position_relative_callback(self, relative_pose):

        ang_z = self.euler_from_pose(self.current_pose)[2]
        print(ang_z)
        delta_x = relative_pose.position.x*np.cos(ang_z)-relative_pose.position.y*np.sin(ang_z)
        delta_y = relative_pose.position.y*np.cos(ang_z)+relative_pose.position.x*np.sin(ang_z)

        self.goal_pose.position.x += delta_x
        self.goal_pose.position.y += delta_y

        self.goal_pose.position.z += relative_pose.position.z
        new_z_ang = self.euler_from_pose(self.goal_pose)[2] + self.euler_from_pose(relative_pose)[2]
        if new_z_ang < -np.pi: new_z_ang+= np.pi
        if new_z_ang > np.pi: new_z_ang-= np.pi
        quarterion = tf.transformations.quaternion_from_euler(0,0,new_z_ang)
        self.goal_pose.orientation.x = quarterion[0]
        self.goal_pose.orientation.y = quarterion[1]
        self.goal_pose.orientation.z = quarterion[2]
        self.goal_pose.orientation.w = quarterion[3]
        self.pid_setpoint(self.goal_pose)
        print(self.goal_pose)
        self.control_mode = "position"
        self.count_aligned = 0
    # def position_relative_callback(self, new_goal_pose):
    #     self.goal_pose.position.x += new_goal_pose.position.x
    #     self.goal_pose.position.y += new_goal_pose.position.y
    #     self.goal_pose.position.z += new_goal_pose.position.z
    #     new_z_ang = self.euler_from_pose(self.goal_pose)[2] + self.euler_from_pose(new_goal_pose)[2]
    #     if new_z_ang < -np.pi: new_z_ang+= np.pi
    #     if new_z_ang > np.pi: new_z_ang-= np.pi
    #     quarterion = tf.transformations.quaternion_from_euler(0,0,new_z_ang)
    #     self.goal_pose.orientation.x = quarterion[0]
    #     self.goal_pose.orientation.y = quarterion[1]
    #     self.goal_pose.orientation.z = quarterion[2]
    #     self.goal_pose.orientation.w = quarterion[3]

    def current_pose_callback(self, current_pose):
        self.current_pose = current_pose
        
        self.current_pose_np = ros_numpy.numpify(current_pose)
        self.current_z_ang = self.euler_from_pose(current_pose)[2]
        self.positioning_vel = self.calculate_vel(self.current_pose)       


    def velocity_callback(self, goal_vec): #Point
        vel = Twist()
        vel.linear.x = goal_vec.x
        vel.linear.y = goal_vec.y
        vel.linear.z = goal_vec.z
        self.setpoint_velocity_pub.publish(vel)
        rospy.loginfo("got velocity goal")

    def parameters_callback(self, config, level):

        rospy.loginfo("""Reconfigure Request: \n Running: {running}\n P_x {P_x} I_x {I_x} D_x {D_x} \n P_y {P_y} I_y {I_y} D_y {D_y} \n P_z {P_z} I_z {I_z} D_z {D_z} \n P_ang {P_ang} I_ang {I_ang} D_ang {D_ang} \n
                                        \n mult_P_x {mult_P_x} mult_I_x {mult_I_x} mult_D_x {mult_D_x} \n mult_P_y {mult_P_y} mult_I_y {mult_I_y} mult_D_y {mult_D_y} \n mult_P_z {mult_P_z} mult_I_z {mult_I_z} mult_D_z {mult_D_z}\n mult_P_ang {mult_P_ang} mult_I_ang {mult_I_ang} mult_D_ang {mult_D_ang}""".format(**config))

        self.pid_x.set_PID_constants(config.P_x*10**config.mult_P_x, config.I_x*10**config.mult_I_x, config.D_x*10**config.mult_D_x)
        self.pid_y.set_PID_constants(config.P_y*10**config.mult_P_y, config.I_y*10**config.mult_I_y, config.D_y*10**config.mult_D_y)
        self.pid_z.set_PID_constants(config.P_z*10**config.mult_P_z, config.I_z*10**config.mult_I_z, config.D_z*10**config.mult_D_z)
        self.pid_ang.set_PID_constants(config.P_ang*10**config.mult_P_ang, config.I_ang*10**config.mult_I_ang, config.D_ang*10**config.mult_D_ang)
        self.running = config.running
        return config

    # ------- service handles ----------
    def set_calibrate_pid(self, request):
        assert isinstance(request, SetBoolRequest)
        self.calibrate_pid = request.data
        srv = Server(ControlConfig, self.parameters_callback)
        return SetBoolResponse(True, "calibrate_pid is now : "+str(self.calibrate_pid))

    # ------ control methods -----------
    def pickup_box(self,data):
        """performs the fast routine of diving, going front and up to pick up a box"""
        rospy.loginfo("PICKUP BOX")
        vel = Twist()
        vel.linear.x = 1.5
        vel.linear.y = 0
        vel.linear.z = 0
        self.setpoint_velocity_pub.publish(vel)
        rospy.sleep(0.25)

        vel.linear.x = 0 #stop
        vel.linear.y = 0
        vel.linear.z = 0
        self.setpoint_velocity_pub.publish(vel)

    def euler_from_pose(self, pose):
        quarterion = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        return tf.transformations.euler_from_quaternion(quarterion)


    def align_camera(self):
        self.camera_angle.angular.x = 0
        self.camera_angle.angular.y = 3
        self.camera_angle.angular.z = 0
        self.camera_angle_pub.publish(self.camera_angle)

    def calculate_vel(self,pose):
        v_x_raw, v_y_raw, v_z, v_ang = self.pid_update(pose)
        ang_z = self.euler_from_pose(pose)[2]
        v_x = np.cos(ang_z)*v_x_raw+np.sin(ang_z)*v_y_raw
        v_y = np.cos(ang_z)*v_y_raw-np.sin(ang_z)*v_x_raw
        # v_x, v_y, v_z = np.dot(self.current_pose_np[:3,:3],np.array([v_x,v_y,v_z])).tolist()
        return [v_x, v_y, v_z, v_ang]
    def reset_pid(self):
        # self.pid_setpoint(self.current_pose)
        self.pid_x.setIntegrator(0)
        self.pid_y.setIntegrator(0)
        self.pid_z.setIntegrator(0)
        self.pid_ang.setIntegrator(0)
        

    def pid_setpoint(self, goal_pose):
        self.pid_x.setPoint(goal_pose.position.x)
        self.pid_y.setPoint(goal_pose.position.y)
        self.pid_z.setPoint(goal_pose.position.z)
        ang_z = self.euler_from_pose(goal_pose)[2]
        self.pid_ang.setPoint(ang_z)

    def pid_update(self, new_pose):
        #linear update
        vel_x = self.pid_x.update(new_pose.position.x)
        vel_y = self.pid_y.update(new_pose.position.y)
        vel_z = self.pid_z.update(new_pose.position.z)
        #angular update
        new_ang_z = self.euler_from_pose(new_pose)[2]
        #check shortest way
        goal_ang = self.pid_ang.getPoint() 
        if new_ang_z - goal_ang > np.pi:
            new_ang_z -= 2*np.pi
        elif new_ang_z - goal_ang < -np.pi:
            new_ang_z += 2*np.pi
        vel_ang = self.pid_ang.update(new_ang_z)
        return np.array([vel_x,vel_y,vel_z,vel_ang])
    def pid_get_error(self):
        return [self.pid_x.getError(),self.pid_y.getError(),self.pid_z.getError(),self.pid_ang.getError()]

    def check_aligment(self):
        e = self.pid_get_error()

        if abs(e[0]) > self.precision[0] or abs(e[1]) > self.precision[1] or abs(e[2]) > self.precision[2] or abs(e[3]) > self.precision[3]:
            self.count_aligned = 0
        else:
            self.count_aligned += 1
            print(e)

        if self.count_aligned > 5:
            rospy.loginfo("ALIGNED!!")
            self.aligned.publish(True)
    
    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                
                self.check_aligment()

                adjusted_vel = Twist()
                adjusted_vel.linear.x = self.positioning_vel[0]
                adjusted_vel.linear.y = self.positioning_vel[1]
                adjusted_vel.linear.z = self.positioning_vel[2]
                adjusted_vel.angular.z = self.positioning_vel[3]
                

                self.setpoint_velocity_pub.publish(adjusted_vel)

                print(adjusted_vel)
                print(self.pid_get_error())
            self.current_pose_pub.publish(self.current_pose)
            self.rate.sleep()


if __name__ == "__main__":
    c = vso_controler()
    c.run()
