# jsk_magnistartup

## SetUp (Running following commands in the first time within the robot)

### fix ROS_HOSTNAME within the robot

Edit /etc/ubiquity/env.sh
```
  #!/bin/sh
- #export ROS_HOSTNAME=$(hostname).local
+ export ROS_HOSTNAME=$(hostname)
  export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
```

### Setup pigpid.services

see https://github.com/UbiquityRobotics/pi_sonar,
```
wget https://raw.githubusercontent.com/joan2937/pigpio/master/util/pigpiod.service
sudo cp pigpiod.service /etc/systemd/system
sudo systemctl enable pigpiod.service
sudo systemctl start pigpiod.service
```

### add jsk_magni.service

Add `/etc/systemd/system/jsk-magni-startup.service` file
```
[Unit]
Requires=roscore.service
PartOf=roscore.service
After=magni-base.service
[Service]
Type=simple
User=ubuntu
ExecStart=/usr/sbin/jsk-magni-startup
[Install]
WantedBy=multi-user.target
```

Add `/usr/sbin/jsk-magni-startup`
```
#!/bin/bash

function log() {
  logger -s -p user.$1 ${@:2}
  }

log info "magni-base: Using workspace setup file /home/ubuntu/catkin_ws/devel/setup.bash"
source /home/ubuntu/catkin_ws/devel/setup.bash

log_path="/tmp"

source /etc/ubiquity/env.sh
log info "magni-base: Launching ROS_HOSTNAME=$ROS_HOSTNAME, ROS_IP=$ROS_IP, ROS_MASTER_URI=$ROS_MASTER_URI, ROS_LOG_DIR=$log_path"

# Punch it.
export ROS_HOME=$(echo ~ubuntu)/.ros
export ROS_LOG_DIR=$log_path
roslaunch jsk_magni_startup magni_bringup.launch &
PID=$!

log info "jsk-magni-startup: Started roslaunch as background process, PID $PID, ROS_LOG_DIR=$ROS_LOG_DIR"
echo "$PID" > $log_path/jsk-magni-startup.pid
wait "$PID"
```

Enable service
```
$ sudo systemctl enable jsk-magni-startup.service
$ sudo systemctl start jsk-magni-startup.service
```

### Add latest codes within catkin_ws

# Update catkin_ws/src
```
- git:
    local-name: demos
    uri: https://github.com/UbiquityRobotics/demos.git
    version: 52136f04c0f39fe6a1001a17e208e5ce5b4cda61
- git:
    local-name: magni_robot
    uri: https://github.com/UbiquityRobotics/magni_robot
    version: 603cae184bcc59b96d59f2fee6029b22725c3c5c
- git:
    local-name: pi_sonar
    uri: https://github.com/UbiquityRobotics/pi_sonar
    version: b84458f909d5febd410b8cd68d552e9aecad685c
```
