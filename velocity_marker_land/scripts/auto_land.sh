#! /bin/bash

# This script works, use `gnome-terminal -x xxx.sh` command

source /opt/ros/kinetic/setup.bash
source /home/odroid/catkin_ws/devel/setup.bash
source /home/odroid/Apriltag_ROS/apriltags_ws/devel/setup.bash

roslaunch velocity_marker_land auto_land.launch
#roslaunch offb_ctrl offbctrl_go.launch
exit 0
