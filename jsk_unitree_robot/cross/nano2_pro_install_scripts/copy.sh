#!/bin/bash

usage() { echo "Usage: $0 [-i <Pro|Air> ]" 1>&2; exit 1; }

TYPE="Pro"
while getopts "t:" o; do
    case "${o}" in
        t)
            TYPE=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))
[ $# -gt 0 ] && usage  # exit if unknown argument found

set -euf -o pipefail

if [[ ${TYPE} == "Pro" ]] ; then
    ip="192.168.123.14"
elif [[ ${TYPE} == "Air" ]] ; then
    ip="192.168.123.13"
fi

ssh-keygen -f "${HOME}/.ssh/known_hosts" -R "${ip}" || echo "OK"
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@${ip} exit

cat rosdep.tgz | ssh unitree@${ip} 'tar -C / -xvzf - '
