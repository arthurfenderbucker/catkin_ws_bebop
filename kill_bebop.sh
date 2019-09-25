#!/bin/sh

#please change the path to your workspace path
#add the following comand to an keyboard shortcut (land_drone):
#gnome-terminal -e "bash -c 'bash <HOME path>/catkin_ws_bebop/kill_bebop.sh land'"
# and to a different one (kill_drone):
# gnome-terminal -e "bash -c 'bash <HOME path>/catkin_ws_bebop/kill_bebop.sh reset'"
echo [ $1 = 'land' ] 
if [ $1 = 'land' ] ; then
    echo "land"
    touch /home/jose/catkin_ws_bebop/land_bebop.txt
    sleep 0.01
    rm /home/jose/catkin_ws_bebop/land_bebop.txt
else
    touch /home/jose/catkin_ws_bebop/kill_bebop.txt
    sleep 0.01
    rm /home/jose/catkin_ws_bebop/kill_bebop.txt
fi
