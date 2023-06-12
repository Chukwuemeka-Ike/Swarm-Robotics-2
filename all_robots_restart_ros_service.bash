#!/bin/bash
HOSTS=("192.168.1.101" "192.168.1.102" "192.168.1.103" "192.168.1.104")
USERNAMES=("administrator" "administrator" "administrator" "administrator")
PASSWORDS=("clearpath" "clearpath" "clearpath" "clearpath")

SCRIPTS=(
    "echo clearpath | sudo -S systemctl restart ros.service;"
    "echo clearpath | sudo -S systemctl restart ros.service;"
    "echo clearpath | sudo -S systemctl restart ros.service;"
    "echo clearpath | sudo -S systemctl restart ros.service;")

echo ${SCRIPTS}
for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R ${HOSTS[i]}
    # sudo apt-get install sshpass
    sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
    # ssh -o StrictHostKeyChecking=no -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
done

# echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc; 
