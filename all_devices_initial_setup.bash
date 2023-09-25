#!/bin/bash

INSTALL_SCRIPT=""

# Decide which config and install script to source based on script argument
if [[ $1 == "computers" ]]; then
    source ~/catkin_ws_swarm2/computers.sh
    INSTALL_SCRIPT="computers_install.bash"
elif [[ $1 == "robots" ]]; then
    source ~/catkin_ws_swarm2/robots.sh
    INSTALL_SCRIPT="robots_install.bash"
elif [[ $1 == "all" ]]; then
    source ~/catkin_ws_swarm2/computers.sh
    source ~/catkin_ws_swarm2/robots.sh
    # Maybe you want to run both install scripts one after the other or decide on a different logic here
else
    echo "Invalid argument. Use computers, robots, or all."
    exit 1
fi

for i in "${!HOSTS[@]}"; do
    echo "------------"
    echo "Setting up: ${HOSTS[i]} as ${USERNAMES[i]}"

    # Remove any previous SSH known hosts entries
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"

    # Copy the install script to the remote machine
    sshpass -p ${PASSWORDS[i]} scp -o StrictHostKeyChecking=no -o HostKeyAlgorithms=ssh-rsa "$HOME/catkin_ws_swarm2/$INSTALL_SCRIPT" "${USERNAMES[i]}@${HOSTS[i]}:/tmp"

    # Run the install script on the remote machine
    sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms=ssh-rsa -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]} "bash /tmp/$INSTALL_SCRIPT"
done
