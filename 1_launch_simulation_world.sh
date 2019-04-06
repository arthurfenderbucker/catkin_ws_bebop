unset PYTHONPATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/catkin_ws_bebop/src/simulation/models
sphinx  ~/catkin_ws_bebop/src/simulation/worlds/imav_2019.world ~/catkin_ws_bebop/drones/bebop2nolan.drone
