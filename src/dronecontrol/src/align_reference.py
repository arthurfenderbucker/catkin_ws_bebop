#!/usr/bin/env python
import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Vector3
from std_msgs.msg import Bool, Float32
from PID import PID

class adjust_position():
    image_shape = np.array([856,480])
    bool_align = False
    last_ref_point_time = 0
    
    goal_point = np.append( image_shape.copy() / 2, 135) # center of the image as default
    current_point = goal_point.copy() 
    precision = np.array([40,40,5]) # pixels

    count_aligned = 0

    # vertical angle between camera's perpenficular vector and horizontal plane
    camera_angle = 0.0
    
    tracking = False
    last_ref_point_time = 0.0

    pid_x_calibration_data_p = [0.0014,0.0018,0.18,0.18] #value and distance
    pid_x_calibration_data_i = [0.00002,0.0002,0.000004,0.000004] #value and distance
    pid_x_calibration_data_d = [0.35,0.7,1.4,1.4] #value and distance

    trust_factor = 0.5

    pid_x_calibration_data_dist = [ 3 ,4, 5  ,30] # [ 2.16 ,3.52,4.36,30]
    # (A,B) = np.polyfit(pid_x_calibration_data_p, pid_x_calibration_data_dist, 1)
    # print
    

    # #40 dist 3m 255
    # pid_x = PID(P=0.0014,I=0.00002,D=0.35)

    #40 dist 2m 241
    pid_x = PID(P=0.0018,I=0.00002,D=0.7)

    # 40cm dist 1m 210
    pid_x = PID(P=0.0018,I=0.000004,D=1.4)

    # pid_x = PID(P=0.0018,I=0.000004,D=1.4)

    pid_x = PID(P=0.0003,I=0.000001,D=0.02)
    pid_y = PID(P=0.004, I=0.00001,D=0.02)
    pid_z = PID(P=0.0004,I=0.000001,D=0.004)

    pid_x.setPoint(goal_point[0])
    pid_y.setPoint(goal_point[1])
    pid_z.setPoint(goal_point[2])
    
    target_radium_real_size = 0.23
    f = 320 #equivalent focal length

    def __init__(self):

        rospy.init_node('align_reference', anonymous=True)
        self.rate = rospy.Rate(25) # 20hz

        self.running = rospy.get_param('~running',True)


        rospy.Subscriber(
                    "/control/align_reference/ref_point", Point, self.point_callback, queue_size=1)
        rospy.Subscriber(
                    "/control/align_reference/set_image_shape", Point, self.set_image_shape, queue_size=1)
        rospy.Subscriber(
                    "/control/align_reference/set_goal_point", Point, self.set_goal_point, queue_size=1)
        
        rospy.Subscriber(
                    "/control/align_reference/set_camera_angle", Float32, self.set_camera_angle, queue_size=1)
        
        self.running_sub= rospy.Subscriber(
            "control/align_reference/set_runnig_state", Bool, self.set_runninng_state, queue_size=1)
        
        self.setpoint_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.aligned = rospy.Publisher(
            "/control/align_reference/aligned", Bool, queue_size=1)
        self.pid_error_pub = rospy.Publisher(
            "/control/align_reference/pid_error", Point, queue_size=1)

        

    # ================ topic callback functions ===================

    def point_callback(self, data): #point
        self.last_ref_point_time = rospy.get_time()
        self.bool_align = True

        self.current_point = np.array([data.x,data.y,data.z])
        
    def set_image_shape(self,data): #point
        self.image_shape =  np.array([data.x,data.y])
        self.current_point = self.goal_point.copy()
    def set_precision(self, data): #point
        self.precision =  np.array([data.x,data.y,data.z])
    def set_goal_point(self,data): #point
        self.goal_point = np.array([data.x,data.y,data.z])
        self.pid_x.setPoint(data.x)
        self.pid_y.setPoint(data.y)
        self.pid_z.setPoint(data.z)
        self.bool_align = False
        self.count_aligned = 0

    def set_runninng_state(self,boolean_state):
        self.running = boolean_state.data

    def set_camera_angle(self,data):#float
        self.camera_angle = data.data
    # ================== control functions ========================
    def pub_vel(self,vel):
        setted_vel = Twist()

        # note: add angle correction
        setted_vel.linear.x = vel[2] *self.trust_factor
        setted_vel.linear.y = vel[0] *self.trust_factor
        setted_vel.linear.z = vel[1] *self.trust_factor

        self.setpoint_vel_pub.publish(setted_vel)


    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                #check_reference_lost (no references received in 1 second)
                if rospy.get_time() - self.last_ref_point_time > 1 :
                    
                    #stop drone
                    vel = np.array([0,0,0])
                    self.bool_align = False
                    self.count_aligned = 0
                    self.tracking = False
                    rospy.loginfo("reference lost")
                else:
                    vel = self.calculate_vel()
                    self.tracking = True

                self.pub_vel(vel)
            self.rate.sleep()

    def get_distance(self):
        d = self.f*self.target_radium_real_size/self.current_point[2]
        print(d)
        return d
    def calculate_vel(self):
        print(self.get_distance())
        # for i,k in enumerate(self.pid_x_calibration_data_dist):
        #     if d < k:
        #         self.pid_x.setKp(self.pid_x_calibration_data_p[i])
        #         self.pid_x.setKi(self.pid_x_calibration_data_i[i])
        #         self.pid_x.setKd(self.pid_x_calibration_data_d[i])
        #         break
        
        vel_raw = np.array([0.0,0.0,0.0])

        vel_raw[0] = self.pid_x.update(self.current_point[0])
        vel_raw[1] = self.pid_y.update(self.current_point[1])
        vel_raw[2] = self.pid_z.update(self.current_point[2])
        
        # if abs(self.pid_x.getError()) > self.precision[0] or abs(self.pid_y.getError()) > self.precision[1]:
        #     vel_raw[2] = 0
        # vel_raw[2] = 0
        cos = np.cos(self.camera_angle)
        sin = np.sin(self.camera_angle)

        camera_tf = np.array([[ 1.0,0.0,0.0],
                              [ 0.0,cos,-sin],
                              [ 0.0,sin,cos]])
        # vel = vel_raw
        vel = np.dot(camera_tf,vel_raw)
        e = Point()
        e.x = self.pid_x.getError()
        e.y = self.pid_y.getError()
        e.z = self.pid_z.getError()
        print(e.x,e.y,e.z)
        self.pid_error_pub.publish(e)
        print(vel)
        if abs(e.x) > self.precision[0] or abs(e.y) > self.precision[1] or abs(e.z) > self.precision[2]:
            self.bool_align = False
            self.count_aligned = 0
        elif self.tracking:
            self.count_aligned += 1

        if self.count_aligned > 3:
            rospy.loginfo("ALIGNED!!")
            
            self.bool_align = True
            self.aligned.publish(True)
        
        return vel

    
def main():

    aligner =  adjust_position()
    aligner.run()

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")

if __name__ == "__main__":
    main()
