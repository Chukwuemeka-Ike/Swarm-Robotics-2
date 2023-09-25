#!/usr/bin/env bash
# CUSTOM RELATED
sudo apt-get install -y sshpass;

sudo apt-get install -y ros-noetic-rqt-ez-publisher;
sudo apt-get install -y spacenavd;
sudo apt-get install -y ros-noetic-spacenav-node;
sudo apt-get install -y ros-noetic-tf2-sensor-msgs;
sudo apt-get install -y ros-noetic-imu-tools;
sudo apt-get install -y ros-noetic-imu-pipeline; # for imu_transformer
sudo apt-get install -y ros-noetic-navigation; # for navigation stack
sudo apt-get install -y ros-noetic-visualization-tutorials; # rviz python bindings.

# PYTHON RELATED
sudo apt-get install -y python3-pip;
pip3 install pyserial;
pip3 install quadprog;
pip3 install pandas;
pip3 install pygame;
pip3 install scipy;
pip3 install numpy==1.21; # needed to resolve the issue "AttributeError: module 'numpy' has no attribute 'typeDict'"
pip3 install shapely; # needed to calculate the swarm footprint polygon and costmap parameter updater functions
pip3 install matplotlib==3.7.3;
pip3 install ortools==9.7.2996;

# DINGO RELATED
sudo apt-get install -y ros-noetic-dingo-desktop;

sudo apt-get install -y ros-noetic-dingo-simulator;

sudo apt install ros-noetic-dingo-navigation;


## Building Steps
cd;
mkdir catkin_ws_swarm2;
cd catkin_ws_swarm2;
rm -rf {*,.*};

git clone https://github.com/burakaksoy/Swarm-Robotics-2.git .;
cd src;
git clone https://github.com/burakaksoy/AssistiveRobot-SimulationFiles.git; # only on DESKTOP
git clone https://github.com/burakaksoy/RVizMeshVisualizer.git; # only on DESKTOP
git clone https://github.com/burakaksoy/uwb_gazebo_plugin; # only on DESKTOP
git clone https://github.com/burakaksoy/multiRobotPlanner.git # only on DESKTOP. CK's repo, but cloning my for fork stable testing

cd ..;
catkin_make -DCATKIN_BLACKLIST_PACKAGES='dingo_base;dingo_customization' -DCMAKE_BUILD_TYPE=Release; # on DESKTOP computer 
# catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;arm_gui;arm_msgs;arm_utils;machine_manager;robot_assigner;task_scheduler;ticket_manager' -DCMAKE_BUILD_TYPE=Release; # on Physical Robots
source devel/setup.bash;