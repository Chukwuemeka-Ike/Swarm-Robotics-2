#!/bin/bash

# Decide which config to source based on script argument
if [[ $1 == "computers" ]]; then
    source ~/catkin_ws_swarm2/computers.sh
elif [[ $1 == "robots" ]]; then
    source ~/catkin_ws_swarm2/robots.sh
elif [[ $1 == "all" ]]; then
    source ~/catkin_ws_swarm2/computers.sh
    source ~/catkin_ws_swarm2/robots.sh
else
    echo "Invalid argument. Use computers, robots, or all."
    exit 1
fi

COMMON_SCRIPT="cd ~/catkin_ws_swarm2; 
               git reset --hard; git pull;
               cd ~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles;
               git reset --hard; git pull;
               cd ~/catkin_ws_swarm2/src/RVizMeshVisualizer;
               git reset --hard; git pull;
               cd ~/catkin_ws_swarm2/src/uwb_gazebo_plugin;
               git reset --hard; git pull;
               cd ~/catkin_ws_swarm2/src/multiRobotPlanner;
               git reset --hard; git pull;"

for i in "${!HOSTS[@]}"; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo "${USERNAMES[i]}"
    # echo ${PASSWORDS[i]}
    echo "$COMMON_SCRIPT"
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"
    # sudo apt-get install sshpass
    sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "$COMMON_SCRIPT"
    # ssh -o StrictHostKeyChecking=no -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
done
