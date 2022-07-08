# This file is robot-specific bash configuration used by robot systemd servies
# This file must be at /var/lib/robot/config.bash

# Network interfaces
export IF_ETH="noethernetdevice"
export IF_WIFI="nowifidevice"
export IF_LTE="noltedevice"

# ROS_IP and ROS_MASTER_URI
WIFI_AP_IP=10.42.0.1
rossetmaster $WIFI_AP_IP
rossetip $WIFI_AP_IP

# Robot Name
# export ROBOT_NAME=

# Speaker and Microphone device settings
# export DEVICE_SPEAKER='hw:1,0'
# export DEVICE_MICROPHONE='hw:1,0'

# ROS CONSOLE
export ROSCONSOLE_FORMAT='[${severity}] [WallTime: ${time}] [node:${node}] [func:${function}]: ${message}'

# Config files
# export APP_SCHEDULE_YAML=/path/to/app_schedule.yaml

# Credentials
# export GOOGLE_APPLICATION_CREDENTIALS=/path/to/service_account_json_file
# export DIALOGFLOW_PROJECT_ID=<your dialogflow project id>
# export GOOGLE_DRIVE_SETTINGS_YAML=/path/to/pyDrive_setting_yaml
# export ROSWWW_BASIC_KEYS_YAML=/path/to/roswww_basic_keys.yaml

# Home configuration
# export USE_DOCKING_STATION="<boolean>"
# export DOCKING_STATION_ID=520
# export SPOT_HOME_ID=eng2_73b2
