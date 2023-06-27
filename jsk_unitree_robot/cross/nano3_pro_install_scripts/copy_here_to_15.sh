#!/bin/bash

set -euf -o pipefail

LOCAL_DATA=15-local.tgz
if [ ! -e ${LOCAL_DATA} ]; then
    echo "Could not find ${LOCAL_DATA}"
    exit
fi

set -x
ssh-keygen -f "${HOME}/.ssh/known_hosts" -R "192.168.123.15" || echo "OK"
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.15 exit
sshpass -p 123 ssh -t unitree@192.168.123.15 ls -al /usr/local/
sshpass -p 123 ssh -t unitree@192.168.123.15 ls -al /usr/local/bin
sshpass -p 123 ssh -t unitree@192.168.123.15 ls -al /usr/local/lib
sshpass -p 123 ssh -t unitree@192.168.123.15 ls -al /usr/local/lib/python3.6/dist-packages
sshpass -p 123 ssh -t unitree@192.168.123.15 ls -al /home/unitree/.local/bin /home/unitree/.local/lib

## manyally change /etc/sudoers
## # %sudo ALL=(ALL:ALL) ALL
## %sudo ALL=(ALL) NOPASSWD: ALL

### remove /usr/local/bin/pcl_ binary to increase free space
sshpass -p 123 ssh -t unitree@192.168.123.15 "ls /usr/local/bin/pcl_*" && \
    ssh unitree@192.168.123.15 "tar -cvz /usr/local/bin/pcl_*" 2> >(tee 15-pcl.log) > 15-pcl.tgz && \
    sshpass -p 123 ssh -t unitree@192.168.123.15 "sudo rm -f /usr/local/bin/pcl_*"

cat ${LOCAL_DATA} | ssh -t unitree@192.168.123.15 sudo tar -C / --keep-old-files -xvzf - || echo "failed, but continue"
sshpass -p 123 ssh -t unitree@192.168.123.15 sudo ldconfig
