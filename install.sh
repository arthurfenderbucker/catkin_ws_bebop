
echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5

sudo apt-get update

sudo apt-get install mesa-utils

sudo apt-get install parrot-sphinx


echo "source ~/catkin_ws_bebop/setup.sh" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/catkin_ws_bebop/src/simulation/models" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=$HOME/catkin_ws_bebop/devel${GAZEBO_PLUGIN_PATH:+:$$}" >> ~/.bashrc

source ~/.bashrc
