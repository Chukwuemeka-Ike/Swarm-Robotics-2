#!/usr/bin/bash
sleep 1s;

gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; killall gzclient && killall gzserver; roscore; exec bash\"";
sleep 1s;

# Launch Gazebo simulation environment
gnome-terminal --tab --title="Sim Highbay" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_highbay.launch; exec bash\"";
sleep 5s;

# Launches all robots in sim 
gnome-terminal --tab --title="Sim Dingos" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_dingos.launch; exec bash\"";
sleep 5s;

# gnome-terminal --tab --title="Sim Fabric" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_fabric.launch; exec bash\"";
sleep 1s;

# gnome-terminal --tab --title="UI Tablet" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_ui_tablet.launch; exec bash\"";
gnome-terminal --tab --title="UI Desktop" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_ui_desktop.launch; exec bash\"";
sleep 1s;

gnome-terminal --tab --title="Main Swarm Control" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch main_swarm_control.launch; exec bash\"";
sleep 1s;