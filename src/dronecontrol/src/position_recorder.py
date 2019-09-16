#!/usr/bin/env python

import rospy
import json
from geometry_msgs.msg import Point

import rospkg


rospack = rospkg.RosPack()
positions_path = str(rospack.get_path('dronecontrol')+'/config/recorded_routines.json')


with open(positions_path, 'r') as json_data_file:
    try:
        positions_data = json.load(json_data_file)
    except:

        positions_data = {}

decision = 0
pose = []

def pose_callback(data):
    global pose
    pose = [data.x,data.y,data.z]
    
rospy.init_node('position_recorder', anonymous=True)
sub = rospy.Subscriber("/control/current_pose", Point, pose_callback, queue_size=1)

routine_name = "default"
while decision != 2 :
    print("Selected routine: "+routine_name)
    print (' 1: save position \n 2: exit \n 3: change routine\n 4: remove routine')
    decision = input()
    if decision == 1:
        if len(pose)>0: 
            if routine_name in positions_data.keys():
                positions_data[routine_name] = positions_data[routine_name]+[pose]
            else:
                positions_data[routine_name] = [pose]

            with open(positions_path, 'w') as json_data_file:
                json.dump(positions_data, json_data_file)
            print("position saved")
        else:
            print("no new positions have been received yet!\nplease check if the \"control.py\" node is running and if the drone is connected")
    if decision == 3:
        print("existing routines:")
        print(positions_data.keys())
        routine_name = str(raw_input("please write the name of the routine edit or create: "))
        if routine_name in positions_data.keys():
            print("existing routine selected!")    
            print(positions_data[routine_name])
        else:
            print("new routine selected!")
    if decision == 4: #delete routine
        if str(raw_input("are you sure? (y/n):")) == "y":
            positions_data.pop(routine_name,None)
            routine_name = "default"
            print("routine deleted")
    print("\n\n\n")
        