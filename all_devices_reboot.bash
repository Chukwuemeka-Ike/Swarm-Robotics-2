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

for i in "${!HOSTS[@]}"; do
    echo "------------"
    echo "${USERNAMES[i]}"

    # Dynamically construct the reboot command using the password from PASSWORDS array
    SCRIPT="echo ${PASSWORDS[i]} | sudo -S reboot;"

    echo "$SCRIPT"
    
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"
    
    sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "$SCRIPT"
done

# echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc; 
