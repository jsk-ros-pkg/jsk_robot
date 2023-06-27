#! /bin/bash

usage() { echo "Usage: $0 [-a address ] [-p password ] [-d <User|System>]" 1>&2; exit 1; }

TARGET_DIRECTORY=User
PASS=""

while getopts "a:p:d:" o; do
    case "${o}" in
        a)
            NAO_IP=${OPTARG}
            ;;
        p)
            PASS=${OPTARG}
            ;;
        d)
            TARGET_DIRECTORY=${OPTARG}
            [[ ${TARGET_DIRECTORY} == "User" || ${TARGET_DIRECTORY} == "System" ]] || usage
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))
[ $# -gt 0 ] && usage  # exit if unknown argument found


TARGET_MACHINE="${TARGET_MACHINE:-i386}"

set -euf -o pipefail

# Check if ros has been cross-compilled
if [ ! -d "$(pwd)/${TARGET_MACHINE}_${TARGET_DIRECTORY}" ]; then
    echo "ERROR: System directory is not found" 1>&2
    echo "ERROR: please build ros for pepper first" 1>&2
    exit 1
fi

# Get password

# Receive pepper hostname or ip
echo "Automatic install script for Pepper \n"
echo "!!! CAUTION !!! If you modifeid files in the Pepper, it will be removed/overwrited"

function copy_data () {
    user=$1
    hostname=$2

    if [ "${PASS}" == "" ]; then
        echo "Enter password for ${hostname} : "
        read PASS
    fi

    # Check if robot is reachable
    reachability=$(ping -c4 ${hostname} 2>/dev/null | awk '/---/,0' | grep -Po '[0-9]{1,3}(?=% packet loss)') || true
    if [ -z "$reachability" ] || [ "$reachability" == 100 ]; then
        echo "ERROR: ${hostname} unreachable" 1>&2
        exit 2
    fi

    # clear old known_hosts
    ssh-keygen -f "${HOME}/.ssh/known_hosts" -R "${hostname}" || echo "OK"
    sshpass -p $PASS ssh -o StrictHostKeyChecking=no ${user}@${hostname} exit

    # cehck disk space
    echo "Copy ${TARGET_MACHINE}_${TARGET_DIRECTORY} ...."
    echo "==="
    du -sh ${TARGET_MACHINE}_${TARGET_DIRECTORY}
    echo "==="

    echo "  ... to ${user}@${hostname}:/home/nao/${TARGET_DIRECTORY}"
    echo "==="
    sshpass -p $PASS ssh -t ${user}@${hostname} bash -c 'df\ -h'
    echo "==="

    rsync --rsh="/usr/bin/sshpass -p $PASS ssh -o StrictHostKeyChecking=no -l ${user}" -avz --exclude "pr2-read-state.l" --exclude "pr2-send-joints.l" --exclude "logs/*" --delete --exclude "*.pyc" --exclude "*.pyo" --exclude "logs/*" --exclude "etc/rosdep/*" ${TARGET_MACHINE}_${TARGET_DIRECTORY}/ ${hostname}:/home/nao/${TARGET_DIRECTORY}
    set +x

    if [[ "${TARGET_DIRECTORY}" == "System" ]]; then
        sed -i 's@if 0:@if 1:@' i386_System/Python-2.7.17/lib/python2.7/site-packages/rosdep2/sources_list.py
    fi

    if [[ "${TARGET_DIRECTORY}" == "User" ]]; then
        ##  add ls; then run your command ???? https://stackoverflow.com/a/33107011
        sshpass -p $PASS ssh -t ${user}@${hostname} bash -c 'ls; test -e .ros/rosdep/sources.cache/index && exit 0; echo "OK"; source User/user_setup.bash; export ROS_ETC_DIR=User/etc; rosdep init; rosdep update'
        sshpass -p $PASS ssh -t ${user}@${hostname} bash -c 'ls; mkdir -p .local/share/PackageManager/apps/img/html/; cp -r User/src/jsk_robot/jsk_naoqi_robot/jsk_pepper_startup/apps/meeting/image/* .local/share/PackageManager/apps/img/html/'
    fi
}

if [[ "${TARGET_DIRECTORY}" == "System" ]]; then
    set +f
    sed -i 's@/usr/bin/python@/usr/bin/env python@' i386_System/ros1_inst/bin/* || echo "OK"
    set -f
fi

copy_data nao "$NAO_IP"

set +x
echo "==="
echo "Congratulations! Please reset robot and enjoy!!"
echo "==="
