# jsk_fetch_startup

- [jsk_fetch_startup](#jsk_fetch_startup)
  - [SetUp (Running following commands in the first time)](#setup-running-following-commands-in-the-first-time)
    - [Install a udev rule](#install-a-udev-rule)
    - [For realsense](#for-realsense)
    - [supervisor](#supervisor)
    - [cron](#cron)
    - [mongodb](#mongodb)
  - [Teleoperation](#teleoperation)
  - [Maintenance](#maintenance)
    - [re-roslaunch jsk_fetch_startup fetch_bringup.launch](#re-roslaunch-jsk_fetch_startup-fetch_bringuplaunch)
    - [re-roslaunch fetch_bringup fetch.launch](#re-roslaunch-fetch_bringup-fetchlaunch)
    - [Clock Synchronization](#clock-synchronization)
  - [Network](#network)
    - [General description](#general-description)
    - [Case description](#case-description)
    - [Access point](#access-point)
  - [Log](#log)
  - [Apps](#apps)
    - [Note](#note)
    - [Add fetch to rwt_app_chooser](#add-fetch-to-rwt_app_chooser)
    - [Execute demos](#execute-demos)
  - [Administration](#administration)

## SetUp (Running following commands in the first time)

### Install a udev rule
```bash
rosrun jsk_fetch_startup install_udev.sh
```

### For realsense

Before start this, remove `librealsense2` and `realsense2-ros` from ROS repository and set not to be installed.

```bash
sudo apt purge ros-melodic-librealsense2* ros-melodic-realsense*
sudo apt-mark hold ros-melodic-librealsense2 ros-melodic-realsense2-camera
```

After that, please add Intel repository and install `librealsense2` packages. (see [this page](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages))

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt install librealsense2=2.45.0-0~realsense0.4551
sudo apt install librealsense2-dev=2.45.0-0~realsense0.4551
sudo apt-mark hold librealsense2 librealsense2-dev
```

### Install config.bash

JSK fetch system uses some enviroment variables. To set them, copy `config.bash` to `/var/lib/robot/config.bash` and modify it.

```bash
roscd jsk_fetch_startup
sudo cp config/config.bash /var/lib/robot/config.bash
```

descriptions of each variable are below.

- `USE_BASE_CAMERA_MOUNT`
  + Flag for robot model
- `USE_HEAD_BOX`
  + Flag for robot model
- `USE_HEAD_L515`
  + Flag for robot model
- `USE_INSTA360_STAND`
  + Flag for robot model
- `RS_SERIAL_NO_T265`
  + Serial number of Realsense T265 for visual odometry.
  + realsense will not be launched when this is blank.
- `RS_SERIAL_NO_D435_FRONTRIGHT`
  + Serial number of Realsense D435 on the base
  + realsense will not be launched when this is blank.
- `RS_SERIAL_NO_D435_FRONTLEFT`
  + Serial number of Realsense D435 on the base
  + realsense will not be launched when this is blank.
- `RS_SERIAL_NO_L515_HEAD`
  + Serial number of Realsense L515 on the head
  + realsense will not be launched when this is blank.
- `NETWORK_DEFAULT_WIFI_INTERFACE`
  + Wi-Fi network interface for network management scripts (e.g. `network_monitor.py` and `network-log-wifi.sh`)
- `NETWORK_DEFAULT_PROFILE_ID`
  + Network manager profile ID for network management scripts (`network_monitor.py`)
+ `NETWORK_DEFAULT_ROS_INTERFACE`
  + Network interface which is used for ROS connection.
  + `ROS_IP` is set to the IP address of this interface in supervisor jobs.

It is also recommended to add lines below to each users's bashrc in the robot PC.

```bash
source /var/lib/robot/config.bash
rossetmaster localhost
rossetip $NETWORK_DEFAULT_ROS_INTERFACE
```

### supervisor

Important jobs for fetch operation are managed by supervisor.

Here is a list of jobs that are managed by supervisor.

- roscore

    Start roscore

- robot

    Launch Minimum ROS programs to run fetch

- jsk-fetch-startup

    Launch ROS programs extended by JSK

- jsk-network-monitor

    Restart the network manager automatically if ping does not work.

- jsk-log-wifi

    Monitor network condition

- jsk-app-scheduler

    Scheduler to launch [app](https://github.com/knorth55/app_manager_utils/tree/master/app_scheduler) at a fixed time

- jsk-object-detector

    Object detection using fetch's head camera and [coral_usb_ros](https://github.com/knorth55/coral_usb_ros)

- jsk-panorama-object-detector:

    Object detection using fetch's 360 camera and [coral_usb_ros](https://github.com/knorth55/coral_usb_ros)

- jsk-human-pose-estimator

    Human pose estimation using fetch's head camera and [coral_usb_ros](https://github.com/knorth55/coral_usb_ros)

- jsk-panorama-human-pose-estimator

    Human pose estimation using fetch's 360 camera and [coral_usb_ros](https://github.com/knorth55/coral_usb_ros)

- jsk-dialog

    Launch [dialogflow_task_exective](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/dialogflow_task_executive)

- jsk-gdrive

    Launch [app](https://github.com/knorth55/app_manager_utils/tree/master/app_uploader) to upload data to Goole Drive

- jsk-dstat

    Monitor fetch's resource using dstat command

- jsk-lifelog

    Launch program to save fetch's [lifelog](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_robot_common/jsk_robot_startup/lifelog)

Install supervisor config files. e.g. `robot.conf`, `jsk_fetch_startup.conf` ...

```bash
su -c 'rosrun jsk_fetch_startup install_supervisor.sh'
```

To show or change supervisor job status, access `supervisor.FETCH_FQDN` by web browser.

Previously, upstart was used, but it has been moved to supervisor. This is because of the convenience of job management via a web browser.

![supervisor_status](https://user-images.githubusercontent.com/19769486/119499716-f142c000-bda1-11eb-9b96-0cfa7e04a1b2.png)

### cron

Install cron jobs for root user and fetch user. e.g. `shutdown`, `update_workspace`.

```bash
su -c 'rosrun jsk_fetch_startup install_cron.sh'
```

### mongodb

```bash
sudo mkdir -p /var/lib/robot/mongodb_store/

# to see the db items from http://lcoalhost/rockmongo
sudo apt-get install apache2 libapache2-mod-php5 php5-mongo
wget "http://rockmongo.com/downloads/go?id=14" -O rockmongo.zip
unzip rockmongo.zip
sudo mv rockmongo-1.1.7 /var/www/html/rockmongo
# manually change following line in /var/www/html/rockmongo/config.php
# $MONGO["servers"][$i]["control_auth"] = false; // true;//enable control users, works only if mongo_auth=false
```

## Teleoperation

For the JSK safe teleop system, please see [data flow diagram of safe_teleop.launch](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_robot_common/jsk_robot_startup#launchsafe_teleoplaunch)

The numbers assigned to the joystick are as follows.

![joystick_numbered](https://user-images.githubusercontent.com/19769486/28101905-889e9cc2-6706-11e7-9981-5704cc29f2b3.png)
![joystick_numbered2](https://user-images.githubusercontent.com/19769486/28101906-88b5f20a-6706-11e7-987c-d94e64ac2cc1.png)

## Maintenance

### re-roslaunch jsk_fetch_startup fetch_bringup.launch

```bash
sudo supervisorctl restart jsk-fetch-startup
```

### re-roslaunch fetch_bringup fetch.launch

```bash
sudo supervisorctl restart robot
```

### [Clock Synchronization](https://github.com/fetchrobotics/docs/blob/0c1c63ab47952063bf60280e74b4ff3ae07fd914/source/computer.rst)

install `chrony` and add ```server `gethostip -d fetch15` offline minpoll 8``` to /etc/chrony/chrony.conf, restart chronyd by `sudo /etc/init.d/chrony restart` and wait for few seconds, if you get

```bash
$ chronyc tracking
Reference ID    : 133.11.216.145 (fetch15.jsk.imi.i.u-tokyo.ac.jp)
Stratum         : 4
Ref time (UTC)  : Wed Oct 26 12:32:56 2016
System time     : 0.000006418 seconds fast of NTP time
Last offset     : 0.003160040 seconds
RMS offset      : 0.003160040 seconds
Frequency       : 11.749 ppm fast
Residual freq   : -137.857 ppm
Skew            : 6.444 ppm
Root delay      : 0.185045 seconds
Root dispersion : 0.018803 seconds
Update interval : 2.1 seconds
Leap status     : Normal
```

it works, if you get `127.127.1.1` for `Reference ID`, something wrong

## Network

### General description

Fetch has wired and wireless network connections.
If we use both of wired and wireless connections as DHCP, DNS holds two IP addresses for same hostname (fetch15 in this case).
This cause problems in network such as ROS communication or ssh connection.

### Case description

If you see the following result, it is OK.

```bash
$ nslookup fetch15.jsk.imi.i.u-tokyo.ac.jp
Server:         127.0.1.1
Address:        127.0.1.1#53

Name:           fetch15.jsk.imi.i.u-tokyo.ac.jp
Address: 133.11.216.145
```

If two or more IP addresses apper, something is wrong.
Please connect display, open a window of network manager.

### Access point

Define access point setting, such as ssid:

```bash
cd /etc/NetworkManager/system-connections
```

## Log

tmuxinator makes it easy to check the important logs of fetch from command line. Currently, it shows the logs of the supervisor jobs.

Install tmuxinator config.

```bash
rosrun jsk_fetch_startup install_tmuxinator.sh
```

Show logs

```bash
tmuxinator log
```

### Show all logs


```bash
tmuxinator log
```

## Administration
- 2016/10/26 add `allow 133.11.216/8` to /etc/chrony/chrony.conf
- 2018/08/26 add `0 10 * * 1-5 /home/fetch/ros/indigo_robot/devel/env.sh rosservice call /fetch15/start_app "name: 'jsk_fetch_startup/go_to_kitchen'"` to crontab
  - `fetch` goes to 73B2 kitchen at 10:00 AM from Monday to Friday.
- 2019/04/19: add `fetch` user in `pulse-access` group.
- 2019/04/19: set `start on runlevel [2345]` in `/etc/init/pulseaudio.conf`.
  - this modification is needed for starting `pulseaudio` in boot.
  - `pulseaudio` is required to register USB speaker on head in boot.
- 2019/04/19: set `env DISALLOW_MODULE_LOADING=0` in `/etc/init/pulseaudio.conf`.
  - this modification is needed for overriding default speaker setting in `/etc/init/jsk-fetch-startup.conf`
  - overriding default speaker setting to use USB speaker on head is done with `pactl set-default-sink $AUDIO_DEVICE` in `/etc/init/jsk-fetch-startup.conf`
- 2019/04/19: launch `jsk_fetch_startup/fetch_bringup.launch` by `fetch` user in `/etc/init/jsk-fetch-startup.conf`
  - some nodes save files by `fetch` user
- 2019/04/19: add arg `launch_teleop` in `/etc/ros/indigo/robot.launch`.
  - We sent PR to upstream [fetchrobotics/fetch_robots PR#40](https://github.com/fetchrobotics/fetch_robots/pull/40).
- 2019/04/19: run `/etc/ros/indigo/robot.launch` with `arg` `launch_teleop:=false`.
  - `teleop` in `/etc/ros/indigo/robot.launch` nodes were conflicted with `teleop` nodes in [jsk_fetch_startup/launch/fetch_teleop.xml](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_fetch_robot/jsk_fetch_startup/launch/fetch_teleop.xml)

## Apps

### Note

You can not run this on Firefox. Please use Google Chrome.

### Add fetch to rwt_app_chooser

1. Access [http://tork-a.github.io/visualization_rwt/rwt_app_chooser](http://tork-a.github.io/visualization_rwt/rwt_app_chooser "website").
  - Be careful to access the site via http, not https, to to enable websocket communication.
  - Modern browsers may automatically redirect from http to https.
1. Click `ADD A ROBOT` button
1. Select `Fetch` at `Robot type`
1. Type `fetch15` at `Robot name`
1. Type `ws://{fetch's IP adress}:9090/` at `Robot URI`
1. Click `ADD ROBOT` button

### Execute demos

1. Click `fetch15` at `Select Robot` window
1. Select task which are shown with icons.

![select_app](https://user-images.githubusercontent.com/19769486/40872010-7d21d2bc-6681-11e8-8c0b-621f199638dd.png)

## Administration

- 2016/10/26 add `allow 133.11.216/8` to /etc/chrony/chrony.conf
- 2018/08/26 add `0 10 * * 1-5 /home/fetch/ros/indigo_robot/devel/env.sh rosservice call /fetch15/start_app "name: 'jsk_fetch_startup/go_to_kitchen'"` to crontab
  - `fetch` goes to 73B2 kitchen at 10:00 AM from Monday to Friday.
- 2019/04/19: add `fetch` user in `pulse-access` group.
- 2019/04/19: set `start on runlevel [2345]` in `/etc/init/pulseaudio.conf`.
  - this modification is needed for starting `pulseaudio` in boot.
  - `pulseaudio` is required to register USB speaker on head in boot.
- 2019/04/19: set `env DISALLOW_MODULE_LOADING=0` in `/etc/init/pulseaudio.conf`.
  - this modification is needed for overriding default speaker setting in `/etc/init/jsk-fetch-startup.conf`
  - overriding default speaker setting to use USB speaker on head is done with `pactl set-default-sink $AUDIO_DEVICE` in `/etc/init/jsk-fetch-startup.conf`
- 2019/04/19: launch `jsk_fetch_startup/fetch_bringup.launch` by `fetch` user in `/etc/init/jsk-fetch-startup.conf`
  - some nodes save files by `fetch` user
- 2019/04/19: add arg `launch_teleop` in `/etc/ros/indigo/robot.launch`.
  - We sent PR to upstream [fetchrobotics/fetch_robots PR#40](https://github.com/fetchrobotics/fetch_robots/pull/40).
- 2019/04/19: run `/etc/ros/indigo/robot.launch` with `arg` `launch_teleop:=false`.
  - `teleop` in `/etc/ros/indigo/robot.launch` nodes were conflicted with `teleop` nodes in [jsk_fetch_startup/launch/fetch_teleop.xml](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_fetch_robot/jsk_fetch_startup/launch/fetch_teleop.xml)
