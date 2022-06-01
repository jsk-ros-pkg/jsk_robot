#!/bin/bash

ssh-keygen -f "${HOME}/.ssh/known_hosts" -R "192.168.123.14" || echo "OK"
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.14 exit

cat rosdep.tgz | ssh unitree@192.168.123.14 'tar -C / -xvzf - '
