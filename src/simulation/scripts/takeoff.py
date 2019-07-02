#!/usr/bin/env python
import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import Imu
from std_msgs.msg import String
from dronecontrol.msg import Vector3D


class takeoff:
    def __init__(self):
        # self.sub_pos = rospy.Subscriber(
        #     "mavros/local_potion/pose", PoseStamped, self.callback)
        rospy.init_node('check_take_off', anonymous=True)

        rospy.Subscriber("/state_machine/state",
                         String, self.state_callback)

        self.pub_condition = rospy.Publisher(
            "/state_machine/takeoff_condition", String, queue_size=1)

        self.pub_position = rospy.Publisher(
            '/controle/position', Vector3D, queue_size=10)

        # rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
        #                  Ardrone3PilotingStateAltitudeChanged, self.callback)

        self.counter = 0
        self.z = 0
        self.pos = Vector3D()
        self.pos.x = 0
        self.pos.y = 0
        self.pos.z = 1.3

    def callback(self, data):
        self.z = data.altitude
        print(data.altitude)
        return

    def state_callback(self, data):
        print("got ")
        print(data.data)
        rospy.loginfo(type(str(data)))
        if (data.data == "takeoff"):

            self.sub_pos = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                                            Ardrone3PilotingStateAltitudeChanged, self.callback)
            self.pub_position.publish(self.pos)

            if self.z > 0.7:
                self.pub_condition.publish("ok")
        elif(self.sub_pos != 0):
            self.sub_pos.unregister()

            # print("exited")

    def execute(self, userdata):
        rospy.loginfo('Executing state takeoff')
        if self.counter < 3:
            self.counter += 1
            return 'flying'
        else:
            return 'erro'


def main():

    print("init")
    t = takeoff()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
