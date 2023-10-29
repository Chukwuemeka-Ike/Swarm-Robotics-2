#!/bin/bash
source ~/catkin_ws_swarm2/robots.sh

for i in "${!HOSTS[@]}"; do
    echo "------------"
    echo "${USERNAMES[i]}"

    # Dynamically construct the reboot command using the password from PASSWORDS array
    SCRIPT="echo ${PASSWORDS[i]} | sudo -S systemctl restart ros.service;"
    # echo clearpath | sudo -S systemctl restart ros.service

    echo "$SCRIPT"
    
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "${HOSTS[i]}"
    
    sshpass -p "${PASSWORDS[i]}" ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l "${USERNAMES[i]}" "${HOSTS[i]}" "$SCRIPT"
done

# echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc; 
