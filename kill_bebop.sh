#!/bin/sh

#please change the path to your workspace path
#add the following comand to an keyboard shortcut:
#gnome-terminal --profile=Unnamed -e "bash -c 'bash /home/arthur/catkin_ws_bebop/kill_bebop.sh'"
touch /home/arthur/catkin_ws_bebop/kill_bebop.txt
sleep 0.01
rm /home/arthur/catkin_ws_bebop/kill_bebop.txt