#!/usr/bin/env python

from geometry_msgs.msg import Twist
import roslib
import os
import rospy
import smach
import smach_ros
from smach_ros import ServiceState

from sm_states import *

# # main


def main():

    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=['land', 'finished', 'succeeded', 'aborted', 'preempted'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('takeoff', takeoff(),
                               transitions={'done': 'land_now',
                                            'error': 'land_now'})

        smach.StateMachine.add('land_now', land_now(),
                               transitions={'done': 'finished'})

        flag_subsm = smach.StateMachine(outcomes=['success', 'error'])
        with flag_subsm:

            smach.StateMachine.add('align_flag', align_flag(),
                                   transitions={'flag_aligned': 'success',
                                                'reference_lost': 'change_view',
                                                'too_many_attempts': 'error'})

            smach.StateMachine.add('change_view', change_view_flag(),
                                   transitions={'done': 'align_flag'})

        smach.StateMachine.add('flag', flag_subsm,
                               transitions={'success': 'face_shelf',
                                            'error': 'land_now'})

        smach.StateMachine.add('face_shelf', face_shelf(),
                               transitions={'done': 'move_to_shelf'}) #goes to the side of the shelf 


        move_to_shelf_subsm = smach.StateMachine(outcomes=['success', 'error'])
        with move_to_shelf_subsm:

            smach.StateMachine.add('align_shelf', align_shelf(),
                                   transitions={'shelf_aligned': 'success',
                                                'reference_lost': 'change_view_shelf',
                                                'too_many_attempts': 'error'})

            smach.StateMachine.add('change_view_shelf', change_view_shelf(),
                                   transitions={'done': 'align_shelf'})


        smach.StateMachine.add('move_to_shelf', move_to_shelf_subsm ,
                               transitions={'success': 'face_boxes',
                                            'error': 'land_now'})

        smach.StateMachine.add('face_boxes', face_boxes(),
                               transitions={'done': 'read_qr_codes'})

        read_qr_codes_subsm = smach.StateMachine(outcomes=['success', 'error'])
        
        with read_qr_codes_subsm:

            smach.StateMachine.add('zigzag_qr_code', zigzag_qr_code(),
                                   transitions={'done': 'success',
                                                'reference_lost': 'change_view_qr_code'})

            smach.StateMachine.add('change_view_qr_code', change_view_qr_code(),
                                   transitions={'done': 'zigzag_qr_code'})


        smach.StateMachine.add('read_qr_codes', read_qr_codes_subsm,
                               transitions={'success': 'land_now',
                                            'error': 'land_now'})




        # smach.StateMachine.add('square', square(),
        #                        transitions={'done': 'land_now', 'error': 'land_now'})

        # smach.StateMachine.add('follow', follow(),
        #                        transitions={'done': 'land_now', 'erro': 'land'})

        

        # Create the sub SMACH state machine
        # sm_sub = smach.StateMachine(outcomes=['no_window', 'yes_window'])
        # # # Open the container
        # with sm_sub:

        #     # Add states to the container
        #     smach.StateMachine.add('find_window', find_window(),
        #                            transitions={'window found': 'yes_window',
        #                                         'not on view': 'change_view',
        #                                         'too many attempts': 'no_window'})

        #     smach.StateMachine.add('change_view', change_view(),
        #                            transitions={'view_changed': 'find_window'})

        # smach.StateMachine.add('WINDOW', sm_sub,
        #                        transitions={'no_window': 'land',
        #                                     'yes_window': 'through_window_state'})
        # smach.StateMachine.add('through_window_state', through_window_state(),
        #                        transitions={'completed window': 'land_now'})
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
