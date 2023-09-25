#!/usr/bin/env bash

export ROS_WS=/home/burak/catkin_ws_swarm2
export ROS_NOETIC=/opt/ros/noetic
source $ROS_NOETIC/setup.bash
source $ROS_WS/devel/setup.bash
export PATH=$ROS_ROOT/bin:$PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WS
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_IP=192.168.1.123

export GAZEBO_MODEL_PATH=~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/models
export GAZEBO_RESOURCE_PATH=~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds
export SDF_PATH=~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/models
export IGN_FILE_PATH=~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds

# TO KILL GAZEBO CLIENT AND SERVER QUICKLY:
alias killg='killall gzclient && killall gzserver && killall rosmaster'

# echo 1234 | sudo -S chmod 666 /dev/ttyACM*

export DISPLAY=:0 # For rviz remote launch

exec "$@"