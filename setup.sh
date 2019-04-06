#setup workspace
source ~/catkin_ws_bebop/devel/setup.bash

#gazebo models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/catkin_ws_bebop/src/simulation/models
export GAZEBO_PLUGIN_PATH=$HOME/catkin_ws_bebop/devel${GAZEBO_PLUGIN_PATH:+:$$}


# ----- setup px4 firmware with gazebo simulation --------

# export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/Firmware/Tools/sitl_gazebo/build
# export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/Firmware/Tools/sitl_gazebo/build


# export SITL_GAZEBO_PATH=$HOME/Firmware/Tools/sitl_gazebo
# export SITL_GAZEBO_PATH=$HOME/src/Firmware/Tools/sitl_gazebo

# export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages

# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/Firmware
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/Firmware/Tools/sitl_gazebo
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/src/Firmware
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/src/Firmware/Tools/sitl_gazebo

# source $HOME/Firmware/Tools/setup_gazebo.bash $HOME/Firmware $HOME/Firmware/build/px4_sitl_default > /dev/null 2>&1
# source $HOME/src/Firmware/Tools/setup_gazebo.bash $HOME/src/Firmware $HOME/src/Firmware/build/px4_sitl_default > /dev/null 2>&1