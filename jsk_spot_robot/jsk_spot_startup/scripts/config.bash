# This file is robot-specific bash configuration used by robot systemd servies
# This file must be at /var/lib/robot/config.bash

# Network interfaces
export IF_ETH="enx70886b8f1b38"
export IF_WIFI="wlxd037458e7f3c"
export IF_LTE="enxf8b7975c750a"

# ROS_IP and ROS_MASTER_URI
WIFI_AP_IP=10.42.0.1
rossetmaster $WIFI_AP_IP
rossetip $WIFI_AP_IP

# ROS CONSOLE
export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}]: ${message}'

# Credentials
# export GOOGLE_APPLICATION_CREDENTIALS=/path/to/service_account_json_file
# export DIALOGFLOW_PROJECT_ID=<your dialogflow project id>
# export GOOGLE_DRIVE_SETTINGS_YAML=/path/to/pyDrive_setting_yaml
