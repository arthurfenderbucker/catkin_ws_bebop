#!/usr/bin/env python

import rospy
import tf
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import rospkg

# to reset the simulation:
# rosservice call /gazebo/reset_world


def main():
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/reset_world")
    print("Got it.")

    reset_world = rospy.ServiceProxy("gazebo/reset_world", Empty)
    reset_world()

    # initial_pose = Pose()
    # initial_pose.position.x = 0
    # initial_pose.position.y = 0
    # initial_pose.position.z = 1

    # rospack = rospkg.RosPack()

    # f = open(rospack.get_path('simulation') + '/models/iris/iris.sdf', 'r')
    # sdff = f.read()
    # spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    # print("ok")
    # delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    # delete_model("iris")

    # spawn_model("iris", sdff,
    #             "robotos_name_space", initial_pose, "world")


if __name__ == '__main__':
    main()
