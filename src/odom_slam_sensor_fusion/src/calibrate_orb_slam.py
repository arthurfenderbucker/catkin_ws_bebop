#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Bool


import json

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import rospkg
import ros_numpy
from sklearn.linear_model import LinearRegression

import rospkg
import tf

rospack = rospkg.RosPack()
positions_path = str(rospack.get_path('odom_slam_sensor_fusion')+'/config/maps/recorded_maps.json')


try:
    with open(positions_path, 'r') as json_data_file:
        calibration_data = json.load(json_data_file)
except:
    calibration_data = {"final_poses":[],"slam_poses":[]}

map_name = "default"

decision = 0
slam_pose = []
count_callbacks = 0
def slam_pose_callback(pose):
    global slam_pose, count_callbacks
    slam_pose = ros_numpy.numpify(pose.pose)
    count_callbacks += 1
    if count_callbacks > 30 and decision == 0:
        count_callbacks =0
        print (".")
        if map_name in calibration_data.keys() and len(calibration_data[map_name])>2:
            print(slam_pose+[1])
            print(np.array(calibration_data[map_name]["tf_matrix"]).T)
            print(np.dot( slam_pose+[1],np.array(calibration_data[map_name]["tf_matrix"]).T))
        
            
    # print(slam_pose)

rospy.init_node('Vel_Control_Node', anonymous=True)
rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, slam_pose_callback)

config_file = rospy.get_param('~config_file',"default.json")


def calculate_map_tf():
    global map_name
    global calibration_data

    x = np.array(calibration_data[map_name]["slam_poses"])
    y = np.array(calibration_data[map_name]["final_poses"])

    

    # reg = LinearRegression().fit(x, y)
    # return np.hstack([reg.coef_,np.expand_dims(reg.intercept_,axis=1)])

while decision != 4 :
    print("Selected map: "+map_name)
    if map_name in calibration_data.keys() and len(calibration_data[map_name])>2:
        print("map already calibrated")
        
    else:
        print("map is not calibrated! Calibration data:")
    if map_name in calibration_data.keys():
        print(calibration_data[map_name]) 
        print("please record at least "+str(3-len(calibration_data[map_name]["final_poses"]))+" more position")

    print (' 1: record this position as the new global position:\n 2: change map\n 3: reset calibration\n 4: exit')
    print("\n dots \".\" means that the Slam is publishing some pose")

    decision = 0
    decision = input()

    if decision == 1:
        if len(slam_pose)>0:
            coord_raw = input("new coord (x,y,z) or (x,y,z,R,P,Y):")
            coord = list(coord_raw)
            if len(coord_raw)==3:
                coord = coord + [0,0,0]
            if not len(coord_raw)==6:
                print("error, make sure you are using comma separeted values")
            else:
                quaternion = tf.transformations.quaternion_from_euler(coord[3], coord[4], coord[5])
                pose = 

                print("orb_coord = ",slam_pose)
                print("final_coord = ",coord)
                if map_name in calibration_data.keys():

                    calibration_data[map_name]["slam_poses"] = calibration_data[map_name]["slam_poses"]+[slam_pose.tolist()]
                    calibration_data[map_name]["final_poses"] = calibration_data[map_name]["final_poses"]+[coord]
                    if len(calibration_data[map_name]["final_poses"])<3:
                        print("please record at least"+str(3-len(calibration_data[map_name]["final_poses"]))+"more position")
                    else:
                        print("calibrated")
                        calibration_data[map_name]["pose_correction_matrix"] = calculate_map_tf().tolist()
                else:
                    calibration_data[map_name]= {"slam_poses":[slam_pose],"final_poses":[coord]}
                    print("please record at least"+str(3-len(calibration_data[map_name]["final_poses"]))+"more position")

                with open(positions_path, 'w') as json_data_file:
                    json.dump(calibration_data, json_data_file)
                print("position saved")
        else:
            print("no positions have been received yet!\nplease check if the node \"orb_slam2_ros\" is running")
    if decision == 2:
        print("existing maps:")
        print(calibration_data.keys())
        map_name = str(raw_input("please write the name of the map edit or create: "))
        if map_name in calibration_data.keys():
            print("existing map selected!")    
            print(calibration_data[map_name])
        else:
            print("new map selected!")
    if decision == 3: #delete map
        if str(raw_input("are you sure? (y/n):")) == "y":
            calibration_data.pop(map_name,None)
            map_name = "default"
            print("map deleted")
    print("\n\n\n")
print("bye bye")