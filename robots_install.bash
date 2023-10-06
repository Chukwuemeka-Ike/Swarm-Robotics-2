#!/usr/bin/env bash

# Check if MY_PASSWORD is set
if [ -z "$MY_PASSWORD" ]; then
    echo "Password not set. Exiting..."
    exit 1
fi

# CUSTOM RELATED
echo $MY_PASSWORD | sudo -S apt-get install -y sshpass;

echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-rqt-ez-publisher;
echo $MY_PASSWORD | sudo -S apt-get install -y spacenavd;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-spacenav-node;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-tf2-sensor-msgs;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-imu-tools;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-imu-pipeline; # for imu_transformer
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-navigation; # for navigation stack
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-visualization-tutorials; # rviz python bindings.

# PYTHON RELATED
echo $MY_PASSWORD | sudo -S apt-get install -y python3-pip;
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
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-dingo-desktop;

echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-dingo-simulator;

echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-dingo-navigation;

# ONLY ON PHYSICAL ROBOTS, NEED TO INSTALL
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-roslint; # needed to build dingo_base package
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-gazebo-msgs;
echo $MY_PASSWORD | sudo -S apt-get install -y ros-noetic-dingo-robot; # AFTER ADDING CLEARPATH KEYS (see https://docs.clearpathrobotics.com/docs/robots/indoor_robots/dingo/tutorials_dingo/#installing-from-debian-packages)

## Building Steps
cd;
mkdir catkin_ws_swarm2;
cd catkin_ws_swarm2;
rm -rf {*,.*};

git clone https://github.com/burakaksoy/Swarm-Robotics-2.git .;
cd src;
# git clone https://github.com/burakaksoy/AssistiveRobot-SimulationFiles.git; # only on DESKTOP
# git clone https://github.com/burakaksoy/RVizMeshVisualizer.git; # only on DESKTOP
# git clone https://github.com/burakaksoy/uwb_gazebo_plugin; # only on DESKTOP
# git clone https://github.com/burakaksoy/multiRobotPlanner.git # only on DESKTOP. CK's repo, but cloning my for fork stable testing
git clone --branch throttle-tf-repeated-data-error https://github.com/BadgerTechnologies/geometry2.git; # to fix tf repeating data warning flooding

cd ..;
source /opt/ros/noetic/setup.bash;
# catkin_make -DCATKIN_BLACKLIST_PACKAGES='dingo_base;dingo_customization' -DCMAKE_BUILD_TYPE=Release; # on DESKTOP computer 
catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;arm_gui;arm_msgs;arm_utils;machine_manager;robot_assigner;task_scheduler;ticket_manager' -DCMAKE_BUILD_TYPE=Release; # on Physical Robots
source devel/setup.bash;