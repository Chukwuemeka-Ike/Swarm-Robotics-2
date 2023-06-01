# Dingo_customization package 
Files in this package includes the all changes that we did on a physical Dingo-O robot for the Swarm 2 project. It is created as a ROS package only for the sake of finding its files with `$(find dingo_customization)` command in other launch files.

## Changed/New Files

The Backups for the edited files are saved in the `/backup` folder.

The edited/new file list as follows:

### Scripts (bash files)

* `/usr/sbin/ros-start`: We specify the `ROS_MASTER` as a remote computer,  define the robot's `ROS_IP`, and comment out the `ROS_HOSTNAME` in this file.
* `/etc/ros/setup.bash`: Includes new environment variable exports for custom name of the robot for namespacing and proper tf frame prefixing. Also has UWB tag id definitions as well as the LIDAR and IMU sensor configurations.

### Launch files

* `/etc/ros/noetic/ros.d/base.launch`: This file is a symbolic link to `/opt/ros/noetic/share/dingo_base/launch/base.launch`.
* `/etc/ros/noetic/ros.d/accessories.launch`: This file is a symbolic link to `/opt/ros/noetic/share/dingo_bringup/launch/accessories.launch`.
* `/opt/ros/noetic/share/dingo_description/launch/description.launch`: We specify the tf prefix of the robot and the uwb tag ids in the robot description file in this launch file. Also the new custom description xacro file of the robot that allows us to do that is specified in this file.
* `/opt/ros/noetic/share/dingo_control/launch/control.launch`: Control parameter file of the omni directional wheels, tf prefixes of the wheels and the odom and robot base frames specified for the knowledge of the joint_publisher and velocity_controllers, robot_localization package configurations with the integration of uwb sensors and custom twist_mux configurations are specified in this file. 

### URDF (.xacro) files

* `...../dingo_customization/dingo-o-prefixed.urdf.xacro`: This is our new file which includes our custom description file of the robot.

<!-- ### Config (.yaml) files

* `...../dingo_customization/config/control_omni.yaml`
* `...../dingo_customization/config/robot_localization.yaml` -->

## Commands to copy in a robot
For example, after ssh'ing into robot `d1`, the following commands copies the necessary files to the correct directories:

```bash
sudo cp ~/catkin_ws_swarm2/src/dingo_customization/d1/scripts/ros-start /usr/sbin/
sudo cp ~/catkin_ws_swarm2/src/dingo_customization/d1/scripts/setup.bash /etc/ros/
sudo cp ~/catkin_ws_swarm2/src/dingo_customization/d1/launch/base.launch /etc/ros/noetic/ros.d/
sudo cp ~/catkin_ws_swarm2/src/dingo_customization/d1/launch/accessories.launch /etc/ros/noetic/ros.d/
```

For other robots (`d2,d3,d4`), ssh into them and execute after replacing the `d1` parts in the commands.
