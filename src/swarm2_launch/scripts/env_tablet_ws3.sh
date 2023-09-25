#!/usr/bin/env bash

export ROS_WS=/home/tablet-ws3/catkin_ws_swarm2
export ROS_NOETIC=/opt/ros/noetic
source $ROS_NOETIC/setup.bash
source $ROS_WS/devel/setup.bash
export PATH=$ROS_ROOT/bin:$PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_IP=192.168.1.98
# echo 1234 | sudo -S chmod 666 /dev/ttyACM*

export DISPLAY=:0 # For rviz remote launch

exec "$@"