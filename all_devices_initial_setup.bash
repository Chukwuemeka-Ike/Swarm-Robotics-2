#!/bin/bash

# Decide which config to source based on script argument
if [[ $1 == "computers" ]]; then
    source ~/catkin_ws_swarm2/computers.sh
    INSTALL_SCRIPT="computers_install.bash"
elif [[ $1 == "robots" ]]; then
    source ~/catkin_ws_swarm2/robots.sh
    INSTALL_SCRIPT="robots_install.bash"
else
    echo "Invalid argument. Use 'computers' or 'robots'"
    exit 1
fi

for i in "${!HOSTS[@]}"; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo "${USERNAMES[i]}"
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"
    
    # Copy the appropriate install script over to the remote device
    sshpass -p "${PASSWORDS[i]}" scp -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 ~/catkin_ws_swarm2/$INSTALL_SCRIPT "${USERNAMES[i]}@${HOSTS[i]}":~/

    # Execute the install script on the remote device
    sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "export MY_PASSWORD=${PASSWORDS[i]}; bash ~/$INSTALL_SCRIPT"
done
