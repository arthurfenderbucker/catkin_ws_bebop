unset PYTHONPATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/catkin_ws_bebop/src/simulation/models
#sphinx  /opt/parrot-sphinx/usr/share/sphinx/worlds/outdoor_5.world ~/catkin_ws_bebop/drones/bebop2nolan.drone
sphinx  ~/catkin_ws_bebop/src/simulation/worlds/imav_2019_2.world ~/catkin_ws_bebop/drones/bebop2nolan.drone
