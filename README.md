# Swarm Project - 2
This is a repository for ARM Swarm Project 2 that utilizes four omnidirectional Clearpath Dingo Robots.

#### Clearpath Dingo all repositories:
https://github.com/dingo-cpr

# The Devices In The Lab:

| Description             | Username      | Hostname (Computer Name) | IP            | Password  | OS           | ROS     |
| ---                     | ---           | ---                      | ---           | ---       | ---          | ---     |
| Pendant Tablet          | tablet        | tablet20                 | 192.168.1.99  | 1234      | Ubuntu 20.04 | Noetic  |
| Main Computer           | razer         | razer-18                 | 192.168.1.100 | 1234      | Ubuntu 18.04 | Melodic |
| Robot 1 (Color)         | dingo01       | dingo01-18               | 192.168.1.101 | 1234      | Ubuntu 18.04 | Melodic |
| Robot 2 (Color)         | dingo02       | dingo02-18               | 192.168.1.102 | 1234      | Ubuntu 18.04 | Melodic |
| Robot 3 (Color)         | dingo03       | dingo03-18               | 192.168.1.103 | 1234      | Ubuntu 18.04 | Melodic |
| Robot 4 (Color)         | dingo04       | dingo04-18               | 192.168.1.104 | 1234      | Ubuntu 18.04 | Melodic |

# Setting up the system

Simply run
```
./TODO
```
<details> 
    <summary>Click to expand</summary>

## Install some dependencies of Dingo Gazebo Simulation and Others
``` bash
sudo apt-get install ros-melodic-lms1xx # ROS driver for the SICK LMS1xx line of LIDARs.
sudo apt-get install ros-melodic-velodyne-simulator # Metapackage of Velodyne LIDAR simulation component
sudo apt-get install ros-melodic-hector-gazebo-plugins
sudo apt-get install ros-melodic-ridgeback-gazebo-plugins
sudo apt-get install ros-melodic-interactive-marker-twist-server
sudo apt-get install ros-melodic-ridgeback-control
sudo apt-get install ros-melodic-rqt-ez-publisher
```

## Building Steps
``` bash
cd;
mkdir catkin_ws_swarm2;
cd catkin_ws_swarm2;
rm -rf {*,.*};

git clone https://github.com/burakaksoy/Swarm-Robotics-2.git .;
cd src;
git clone -b melodic-devel https://github.com/burakaksoy/dingo.git;
git clone https://github.com/burakaksoy/dingo_simulator.git;
git clone https://github.com/burakaksoy/dingo_desktop.git;
git clone https://github.com/burakaksoy/AssistiveRobot-SimulationFiles.git;
git clone https://github.com/burakaksoy/RVizMeshVisualizer.git;
git clone https://github.com/burakaksoy/uwb_gazebo_plugin;

cd ..;
catkin_make -DCMAKE_BUILD_TYPE=Release;
source devel/setup.bash;
```


### In your `~/.bashrc` file, add these:
``` bash
source ~/catkin_ws_swarm2/devel/setup.bash

export GAZEBO_MODEL_PATH=~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/models
export GAZEBO_RESOURCE_PATH=~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds
```

</details> 

# Steps for Gazebo Simulation of Single Dingo-O robot
<details> 
    <summary>Click to expand</summary>

Assuming that you already did the dependancy installations and building in [**Setting up the system**](#setting-up-the-system) section.

(Reference: http://www.clearpathrobotics.com/assets/guides/melodic/dingo/simulation.html)
## Running the simulation
### Gazebo simulation:
``` bash
roslaunch dingo_gazebo empty_world.launch x:=0. y:=0. yaw:=0. config:=front_laser
```
For other config options see: https://github.com/dingo-cpr/dingo/tree/melodic-devel/dingo_description
and create a config file in `.../dingo/dingo_description/urdf/configs/`

Another option for configurations is export their environment variables. For example:
``` bash
export DINGO_OMNI=1
export DINGO_LASER=1
export DINGO_LASER_MODEL='ust10' # or 'lms1xx'
export DINGO_IMU_MICROSTRAIN=1
roslaunch dingo_gazebo empty_world.launch x:=1. y:=0. yaw:=0.
```

### Corresponding RVIZ:
``` bash
roslaunch dingo_viz view_robot.launch
```

### To send simple velocity commands to the robot you can use rqt_ez_publisher:
``` bash
rosrun rqt_ez_publisher rqt_ez_publisher
```
and send messages to `\cmd_vel` topic.

</details> 

# Steps for Gazebo Simulation of Multiple Dingo-O robots

<details>
    <summary>Click to expand</summary>

Assuming that you already did the dependancy installations and building in [**Setting up the system**](#setting-up-the-system) section.

## Running the simulation in Empty World
This command launches the corresponding RVIZ and the rqt_ez_publisher all together. 
``` bash
roslaunch dingo_gazebo empty_world_multi.launch
```
Note that RVIZ TF frames are reported by `robot_localization` package that uses the _odometry_ and _IMU_ information, hence drifts after a while, but it is more realistic in that sense.
<!-- TODO: ADD image here -->
![View in empty world](./.imgs/empty_world_multi.png)

## Running the simulation in CII 8th Floor Lab
This is an example lab environment to visualize the scales of Dingo robots.
This command launches the corresponding RVIZ and the rqt_ez_publisher all together in CII 8th floor lab.
``` bash
roslaunch dingo_gazebo empty_lab_multi.launch
```
Note that RVIZ TF frames are reported by `robot_localization` package that uses the _odometry_ and _IMU_ information, hence drifts after a while, but more realistic.
<!-- TODO: ADD image here -->
![View in CII 8th Floor Lab](./.imgs/empty_lab_multi.png)

## Running the simulation in Empty World with ground truth
This command launches the simulation with ground truth reported TF frames to RVIZ. Again, launching the corresponding RVIZ and the rqt_ez_publisher is embedded all together. 
``` bash
roslaunch dingo_gazebo empty_world_multi_ground_truth.launch
```
Note that RVIZ TF frames are reported by `message_to_tf` package that uses the _ground truth_ data coming from `gazebo_ros_p3d` plugin, hence it is exact representation of the Gazebo World.
Therefore, this command does not launch the Gazebo client GUI to save computational power, but could be re-enabled with gui parameter set to true in the launch file.
<!-- TODO: ADD image here -->
![Empty World with ground truth](./.imgs/empty_world_multi_ground_truth.png)

## Running the simulation in Demonstration Floor - Highbay

This command launches the corresponding RVIZ and the rqt_ez_publisher all together. 
``` bash
roslaunch dingo_gazebo empty_highbay_multi.launch
```
Note that RVIZ TF frames are reported by `robot_localization` package that uses the _odometry_ and _IMU_ information, hence drifts after a while, but it is more realistic in that sense.
<!-- TODO: ADD image here -->
![View in Empty Highbay](./.imgs/empty_highbay_multi.png)

For a simulation that includes the representative workstations and the workers run:
``` bash
roslaunch dingo_gazebo highbay_multi.launch
```
![View in Highbay](./.imgs/highbay_multi.png)

## Running the simulation in Demonstration Floor - Highbay with ground truth
This command launches the simulation with ground truth reported TF frames to RVIZ. Again, launching the corresponding RVIZ and the rqt_ez_publisher is embedded all together. 
``` bash
roslaunch swarm2_launch multi_dingo_sim_with_rviz_and_ez_publisher_highbay.launch
```
Note that RVIZ TF frames are reported by `message_to_tf` package that uses the _ground truth_ data coming from `gazebo_ros_p3d` plugin, hence it is exact representation of the Gazebo World.
Therefore, this command does not launch the Gazebo client GUI to save computational power, but could be re-enabled with gui parameter set to true in the launch file.
<!-- TODO: ADD image here -->
![Highbay World with ground truth](./.imgs/highbay_multi_ground_truth.png)
**Note that above, the world in Gazebo is visualized at RViz! This is only a static image of the world exported as a COLLADA (.dae) file and imported to RViz using [`RVizMeshVisualizer`](https://github.com/burakaksoy/RVizMeshVisualizer). If you make changes to the world file, update the mesh file following the steps in [here](https://github.com/burakaksoy/AssistiveRobot-SimulationFiles/tree/master/lab_gazebo#to-export-world-files-to-a-single-collada-dea).**
**For example, for the Highbay world, if you have already did the installation steps in the link above, run:**
``` bash
cd ~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds;
ign gazebo -v 4 -s -r --iterations 1 highbay_swarm.world
```
**This will update the mesh file corresponding to the `highbay_swarm.world`.**


## Running the simulation in Anchor Industries Representative Plant Floor

**!!!IN PROGRESS, NOT DONE YET!!!**

This command launches the corresponding RVIZ and the rqt_ez_publisher all together. 
``` bash
roslaunch dingo_gazebo plant_floor_multi.launch
```
Note that RVIZ TF frames are reported by `robot_localization` package that uses the _odometry_ and _IMU_ information, hence drifts after a while, but it is more realistic in that sense.
<!-- TODO: ADD image here -->

</details>

# Physical UWB setup
<details> 
    <summary>Click to expand</summary>
    
## Related websites for the Qorvo (DecaWave) UWB module documents
DW1000 [https://www.qorvo.com/products/p/DW1000#documents](https://www.qorvo.com/products/p/DW1000#documents)

DW1001C [https://www.qorvo.com/products/p/DWM1001C#documents](https://www.qorvo.com/products/p/DWM1001C#documents)

DWM1001-DEV [https://www.qorvo.com/products/p/DWM1001-DEV#documents](https://www.qorvo.com/products/p/DWM1001-DEV#documents)

MDEK1001 [https://www.qorvo.com/products/p/MDEK1001#documents](https://www.qorvo.com/products/p/MDEK1001#documents)


## Download the Android DRTLS phone app

[https://www.qorvo.com/products/p/MDEK1001#documents](https://www.qorvo.com/products/p/MDEK1001#documents)

Download DRTLS App : Android Application APK

## Calibration Script
Used to determine the every module's (tags and anchors) offsets based on [this white paper with name: Antenna Delay Calibration of DW1000-Based Products and Systems (Application Note APS014)](https://www.qorvo.com/products/d/da008449).

Set 4 of them an on a nice square with best possible known manual position measurements. 

(3 of them gives only one solution, 4 of them gives a Least Squares solution with RMSE error to have an idea of how accurate the calculated offsets are.)

Take note of the manually measured distances, they are needed in the calibration script.

From the Android app, set all the modules as anchors in the same network and specify the initiator correctly.

Use `antenna_offset_finding.m` MATLAB script in `uwb_matlab_scripts/` directory of this repo to find the offsets of each UWB module. Then set the offsets in `antenna_calibration.yaml` in `src/swarm_launch/config/` folder. Comments of the script should be sufficient to guide you for further details. 

Note: This script would work on Windows 10 but not in Windows 11 as of writing this document. See details [here](https://www.mathworks.com/matlabcentral/answers/1912280-bluetooth-scanning-error-in-windows-11-solutions#answer_1173820)

This script uses the BLE interface of the firmware to communicate with the tags. For further information see section 7 of [DWM1001 Firmware API Guide](https://www.qorvo.com/products/d/da007975)
    
</details>
