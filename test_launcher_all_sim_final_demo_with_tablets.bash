#!/bin/bash
sleep 1s;

gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; killall gzclient && killall gzserver; roscore; exec bash\"";
sleep 1s;
# ================================================================================
# --------------------------------- SIMULATIONS ----------------------------------
# Launch Gazebo simulation environment
gnome-terminal --tab --title="Sim Highbay" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_highbay.launch helper_laptop:=true; exec bash\"";
# gnome-terminal --tab --title="Sim Highbay" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_highbay.launch helper_laptop:=false; exec bash\"";
sleep 5s;

# Launches all robots in sim 
gnome-terminal --tab --title="Sim Dingos" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_dingos.launch helper_laptop:=true; exec bash\"";
# gnome-terminal --tab --title="Sim Dingos" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_dingos.launch helper_laptop:=false; exec bash\"";
sleep 5s;

# gnome-terminal --tab --title="Sim Fabric" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch sim_fabric.launch; exec bash\"";
sleep 1s;

# --------------------------------------------------------------------------------
# ================================================================================
# --------------------------------- HIGH LEVEL -----------------------------------
# Launch high level planner/scheduler
gnome-terminal --tab --title="High level" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch main_high_level.launch; exec bash\"";
sleep 1s;

# Load tickets for the final demonstration
# NOTE: THIS IS TEMPORARY FOR QUICK TESTING,
# TICKETS CAN BE ADDED FROM SUPERVISOR GUI MANUALLY
gnome-terminal --tab --title="Quick Tickets" --command "bash -c \"source ~/.bashrc; export ROS_NAMESPACE=/main; rosrun ticket_manager add_tickets.py; exec bash\"";
sleep 1s;

# Launch Supervisor GUI
gnome-terminal --tab --title="SupervisorGUI" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch main_supervisor_gui.launch; exec bash\"";
sleep 3s;

# --------------------------------------------------------------------------------
# ================================================================================
# --------------------------------- WORKSTATIONS ---------------------------------

# Launch "Loading"     Workstation (Operator GUI - in tablet,  Corresponding Swarm - in main computer)
gnome-terminal --tab --title="Loading WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_loading.launch use_tablet:=true; exec bash\"";
# gnome-terminal --tab --title="Loading WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_loading.launch use_tablet:=false; exec bash\"";
sleep 3s;

# Launch "Mega Stitch" Workstation (Operator GUI - in tablet,  Corresponding Swarm - in main computer)
gnome-terminal --tab --title="MegaStitch WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_megastitch.launch use_tablet:=true; exec bash\"";
# gnome-terminal --tab --title="MegaStitch WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_megastitch.launch use_tablet:=false; exec bash\"";
sleep 3s;

# Launch "RF Welder"   Workstation (Operator GUI - in tablet,  Corresponding Swarm - in main computer)
gnome-terminal --tab --title="RF Welder WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_rf.launch use_tablet:=true; exec bash\"";
# gnome-terminal --tab --title="RF Welder WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_rf.launch use_tablet:=false; exec bash\"";
sleep 3s;

# Launch "Perimeter"   Workstation (Operator GUI - in tablet,  Corresponding Swarm - in main computer)
gnome-terminal --tab --title="Perimeter WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_perimeter.launch use_tablet:=true; exec bash\"";
# gnome-terminal --tab --title="Perimeter WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_perimeter.launch use_tablet:=false; exec bash\"";
sleep 3s;

# Launch "Inspection"  Workstation (Operator GUI - in tablet,  Corresponding Swarm - in main computer)
gnome-terminal --tab --title="Inspection WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_inspection.launch use_tablet:=true; exec bash\"";
# gnome-terminal --tab --title="Inspection WS" --command "bash -c \"source ~/.bashrc; roslaunch swarm2_launch ws_inspection.launch use_tablet:=false; exec bash\"";
sleep 3s;


# --------------------------------------------------------------------------------






