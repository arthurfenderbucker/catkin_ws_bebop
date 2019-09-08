#!/usr/bin/env python
import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Vector3
from std_msgs.msg import Bool
from PID import PID

class adjust_position():
    image_shape = np.array([620,430])
    bool_align = False
    last_ref_point_time = 0
    
    goal_point = np.append( image_shape.copy() / 2, 200 ) # center of the image as default
    current_point = goal_point.copy() 
    precision = np.array([20,20,5]) # pixels

    count_aligned = 0
    # angle relative to camera's perpenficular vector 
    align_angle = 0 
    

    pid_x = PID(P=0.002,I=-0.0001,D=0.0005)
    pid_y = PID(P=0.002,I=-0.0001,D=0.0005)
    pid_z = PID(P=0.002,I=-0.0001,D=0.0005)
    pid_x.setPoint(goal_point[0])
    pid_y.setPoint(goal_point[1])
    pid_z.setPoint(goal_point[2])

    def __init__(self):

        rospy.init_node('align_reference', anonymous=True)
        self.rate = rospy.Rate(20) # 20hz
        rospy.Subscriber(
                    "/control/align_reference/ref_point", Point, self.point_callback, queue_size=None)
        rospy.Subscriber(
                    "/control/align_reference/set_image_shape", Point, self.set_image_shape, queue_size=None)
        
        self.setpoint_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.alginned = rospy.Publisher(
            "/control/align_reference/aligned", Bool, queue_size=1)

        

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

    # ================== control functions ========================
    def pub_vel(self,vel):
        setted_vel = Twist()

        # note: add angle correction
        setted_vel.linear.x = vel[2] 
        setted_vel.linear.y = vel[0]
        setted_vel.linear.z = vel[1]

        self.setpoint_vel_pub.publish(setted_vel)


    def run(self):
        while not rospy.is_shutdown():
            
            #check_reference_lost (no references received in 1 second)
            if rospy.get_time() - self.last_ref_point_time > 1 and self.bool_align:
                #stop drone
                vel = np.array([0,0,0])
                self.bool_align = False
                
                rospy.loginfo("reference lost")
            else:
                vel = self.calculate_vel()

            self.pub_vel(vel)
            self.rate.sleep()

    def calculate_vel(self):
        
        vel = np.array([0.0,0.0,0.0])

        vel[0] = self.pid_x.update(self.current_point[0])
        vel[1] = self.pid_y.update(self.current_point[1])
        vel[2] = self.pid_z.update(self.current_point[2])
        rospy.loginfo(vel)
        return vel

        delta = self.current_point - self.goal_point

        rospy.loginfo(delta)
        
        if abs(delta[0]) > self.precision[0] or abs(delta[1]) > self.precision[1]:
            vel[0] = np.sign(
                delta[0]) * min(np.abs(delta[0]) / 160, 0.1)
            
            vel[1] = np.sign(
                delta[1]) * min(np.abs(delta[1]) / 160, 0.1)
            self.count_aligned = 0

        if abs(delta[2]) > self.precision[2] and abs(delta[0]) < self.precision[1] and abs(delta[1]) < self.precision[0]: 
            #only align the z after the object is centralized
            vel[2] = np.sign(
                delta[2]) * min(np.abs(delta[2]) / 20, 0.2)
            self.count_aligned = 0
        

        self.count_aligned += 1

        if self.count_aligned > 3:
            rospy.loginfo("ALIGNED!!")
            self.bool_align = True
            self.alginned.publish(True)
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