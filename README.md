# Swarm Project - 2
This is a repository for ARM Swarm Project 2 that utilizes four omnidirectional Clearpath Dingo Robots.

#### Clearpath Dingo all repositories:
https://github.com/dingo-cpr

# The Devices In The Lab:

| Description             | Username      | Hostname (Computer Name) | IP            | Password  | OS           | ROS     |
| ---                     | ---           | ---                      | ---           | ---       | ---          | ---     |
| Pendant Tablet          | tablet        | tablet20                 | 192.168.1.99  | 1234      | Ubuntu 20.04 | Noetic  |
| Main Computer           | razer         | razer-18                 | 192.168.1.100 | 1234      | Ubuntu 18.04 | Melodic |
| Robot 1 (Color)         | oarbot_silver | oarbot-silver-P15        | 192.168.1.101 | 1234      | Ubuntu 18.04 | Melodic |

# Setting up the system

Simply run
```
./TODO
```

## Steps for Gazebo Simulation of (Single) Dingo-O robot
(Reference: http://www.clearpathrobotics.com/assets/guides/melodic/dingo/simulation.html)
### Install some dependencies of Dingo Gazebo Simulation
```
sudo apt-get install ros-melodic-lms1xx # ROS driver for the SICK LMS1xx line of LIDARs.
sudo apt-get install ros-melodic-velodyne-simulator # Metapackage of Velodyne LIDAR simulation component
sudo apt-get install ros-melodic-hector-gazebo-plugins
sudo apt-get install ros-melodic-ridgeback-gazebo-plugins
sudo apt-get install ros-melodic-interactive-marker-twist-server
sudo apt-get install ros-melodic-ridgeback-control
```

### Default Gazebo Simulation Building Steps
```
cd ~
mkdir catkin_ws_swarm2

cd catkin_ws_swarm2
mkdir src
catkin_init_workspace src

cd src
git clone <THIS REPO>
git clone -b melodic-devel https://github.com/dingo-cpr/dingo.git
git clone https://github.com/dingo-cpr/dingo_simulator.git
git clone https://github.com/dingo-cpr/dingo_desktop.git

cd ..
catkin_make
source devel/setup.bash
```

### Running the simulation
#### Gazebo simulation:
```
roslaunch dingo_gazebo empty_world.launch x:=0. y:=0. yaw:=0. config:=front_laser
```
For other config options see: https://github.com/dingo-cpr/dingo/tree/melodic-devel/dingo_description
and create a config file in `.../dingo/dingo_description/urdf/configs/`

Another option for configurations is export their environment variables. For example:
```
export DINGO_OMNI=1
export DINGO_LASER=1
export DINGO_LASER_MODEL='ust10' # or 'lms1xx'
export DINGO_IMU_MICROSTRAIN=1
roslaunch dingo_gazebo empty_world.launch x:=1. y:=0. yaw:=0.
```

#### Corresponding RVIZ:
```
roslaunch dingo_viz view_robot.launch
```

#### To send simple velocity commands to the robot you can use rqt_ez_publisher:
```
rosrun rqt_ez_publisher rqt_ez_publisher
```
and send messages to `\cmd_vel` topic.
