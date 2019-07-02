#!/usr/bin/env python
# import mavros
from std_msgs.msg import String
from std_msgs.msg import Empty

from geometry_msgs.msg import Twist
import roslib
import os
import rospy
import smach
import smach_ros
from time import sleep
from smach_ros import ServiceState
# from std_srvs.srv import Empty

pub_state = rospy.Publisher("/state_machine/state",
                            String, queue_size=10)

# define state takeoff


class takeoff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['flying', 'erro'])
        rospy.Subscriber("/state_machine/takeoff_condition",
                         String, self.callback)
        self.takeoff_topic = rospy.Publisher(
            "/bebop/takeoff", Empty, queue_size=1)
        self.condition = ""

    def callback(self, data):
        print(data)
        self.condition = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state takeoff')

        for i in range(2000):
            self.takeoff_topic.publish(Empty())
            rospy.sleep(0.001)

        while self.condition != "ok" and not rospy.is_shutdown():
            pub_state.publish("takeoff")
            sleep(0.1)
            pass
        rospy.loginfo('takeoff done')

        return 'flying'


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

class follow(smach.State):
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

class land(smach.State):
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

# define state find_window

class find_window(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['window found', 'not on view', 'too many attempts'])
        rospy.Subscriber("/state_machine/window/find_condition",
                         String, self.callback)
        self.condition = ""

    def callback(self, data):
        print(data)
        self.condition = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state find window')
        pub_state.publish("find_window")
        while self.condition != "ok" and not rospy.is_shutdown():
            pass
        self.condition = ""
        return 'window found'

# define state change_view


class change_view(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['view_changed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state find window')

        return 'view_changed'

# define state through_window_state


class through_window_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed window'])
        rospy.Subscriber("/state_machine/window/through_condition",
                         String, self.callback)
        self.condition = ""

    def callback(self, data):
        print(data)
        self.condition = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state through window')
        pub_state.publish("through_window")
        while self.condition != "ok" and not rospy.is_shutdown():
            pass
        self.condition = ""
        # pub_state.publish("")
        return 'completed window'

# define state rope


class find_rope(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rope_found'])
        rospy.Subscriber("/state_machine/rope/find_condition",
                         String, self.callback)
        self.condition = ""

    def callback(self, data):
        print(data)
        self.condition = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state find_rope')
        pub_state.publish("find_rope")
        while self.condition != "ok" and not rospy.is_shutdown():
            pass
        self.condition = ""
        rospy.loginfo("exited find_rope")
        return 'rope_found'

# main


def main():

    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=['land', 'finished', 'succeeded', 'aborted', 'preempted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('takeoff', takeoff(),
                               transitions={'flying': 'WINDOW',
                                            'erro': 'land'})

        smach.StateMachine.add('square', square(),
                               transitions={'done': 'land_now', 'erro': 'land'})

        smach.StateMachine.add('follow', follow(),
                               transitions={'done': 'land_now', 'erro': 'land'})

        smach.StateMachine.add('land_now', land(),
                               transitions={'done': 'finished'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['no_window', 'yes_window'])
        # # Open the container
        with sm_sub:

            # Add states to the container
            smach.StateMachine.add('find_window', find_window(),
                                   transitions={'window found': 'yes_window',
                                                'not on view': 'change_view',
                                                'too many attempts': 'no_window'})

            smach.StateMachine.add('change_view', change_view(),
                                   transitions={'view_changed': 'find_window'})

        smach.StateMachine.add('WINDOW', sm_sub,
                               transitions={'no_window': 'land',
                                            'yes_window': 'through_window_state'})
        smach.StateMachine.add('through_window_state', through_window_state(),
                               transitions={'completed window': 'land_now'})
        # smach.StateMachine.add('find_rope', find_rope(),
        #                        transitions={'rope_found': 'reset'})  # finished'})

        # smach.StateMachine.add('reset',
        #                        ServiceState('gazebo/reset_world',
        #                                     Empty),
        #                        transitions={'succeeded': 'takeoff'})

     # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

    # rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
