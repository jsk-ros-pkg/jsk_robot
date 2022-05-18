#!/bin/bash

function usage() {
    echo "Usage: $0 <command>"
    exit 1
}

function start_jobs() {
    sudo supervisorctl start roscore
    sudo supervisorctl start robot
    sudo supervisorctl start jsk-shutdown
    sudo supervisorctl start jsk-rfcomm-bind
    sudo supervisorctl start jsk-object-detector
    sudo supervisorctl start jsk-network-monitor
    sudo supervisorctl start jsk-log-wifi
    sudo supervisorctl start jsk-human-pose-estimator
    sudo supervisorctl start jsk-gdrive
    sudo supervisorctl start jsk-fetch-startup
    sudo supervisorctl start jsk-dstat
    sudo supervisorctl start jsk-dialog
    sudo supervisorctl start jsk-app-scheduler
}

function restart_jobs() {
    sudo supervisorctl restart roscore
    sudo supervisorctl restart robot
    sudo supervisorctl restart jsk-shutdown
    sudo supervisorctl restart jsk-rfcomm-bind
    sudo supervisorctl restart jsk-object-detector
    sudo supervisorctl restart jsk-network-monitor
    sudo supervisorctl restart jsk-log-wifi
    sudo supervisorctl restart jsk-human-pose-estimator
    sudo supervisorctl restart jsk-gdrive
    sudo supervisorctl restart jsk-fetch-restartup
    sudo supervisorctl restart jsk-dstat
    sudo supervisorctl restart jsk-dialog
    sudo supervisorctl restart jsk-app-scheduler
}

function stop_jobs() {
    sudo supervisorctl stop roscore
    sudo supervisorctl stop robot
    sudo supervisorctl stop jsk-shutdown
    sudo supervisorctl stop jsk-rfcomm-bind
    sudo supervisorctl stop jsk-object-detector
    sudo supervisorctl stop jsk-network-monitor
    sudo supervisorctl stop jsk-log-wifi
    sudo supervisorctl stop jsk-human-pose-estimator
    sudo supervisorctl stop jsk-gdrive
    sudo supervisorctl stop jsk-fetch-stopup
    sudo supervisorctl stop jsk-dstat
    sudo supervisorctl stop jsk-dialog
    sudo supervisorctl stop jsk-app-scheduler
}

command=$1

case "$command" in
    start) start_jobs;;
    restart) restart_jobs;;
    stop) stop_jobs;;
    *) usage;;
esac
