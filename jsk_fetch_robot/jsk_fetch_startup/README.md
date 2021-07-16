# jsk_fetch_startup

## SetUp (Running following commands in the first time)


### upstart
```
su -c 'rosrun jsk_fetch_startup install_upstart.sh'
```

### mongodb

```
sudo mkdir -p /var/lib/robot/mongodb_store/

# to see the db items from http://lcoalhost/rockmongo
sudo apt-get install apache2 libapache2-mod-php5 php5-mongo
wget "http://rockmongo.com/downloads/go?id=14" -O rockmongo.zip
unzip rockmongo.zip
sudo mv rockmongo-1.1.7 /var/www/html/rockmongo
# manually change following line in /var/www/html/rockmongo/config.php
# $MONGO["servers"][$i]["control_auth"] = false; // true;//enable control users, works only if mongo_auth=false
```

### Teleoperation

For the JSK safe teleop system, please see [data flow diagram of safe_teleop.launch](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_robot_common/jsk_robot_startup#launchsafe_teleoplaunch)

The numbers assigned to the joystick are as follows.

![joystick_numbered](https://user-images.githubusercontent.com/19769486/28101905-889e9cc2-6706-11e7-9981-5704cc29f2b3.png)
![joystick_numbered2](https://user-images.githubusercontent.com/19769486/28101906-88b5f20a-6706-11e7-987c-d94e64ac2cc1.png)

## Maintenance

### re-roslaunch jsk_fetch_startup fetch_bringup.launch
```
sudo service jsk-fetch-startup restart
```
as of 2016/10/26, it uses launch files under `~k-okada/catkin_ws`

### re-roslaunch fetch_bringup fetch.launch
```
sudo service robot restart
```

### [Clock Synchronization](https://github.com/fetchrobotics/docs/blob/0c1c63ab47952063bf60280e74b4ff3ae07fd914/source/computer.rst)

install `chrony` and add ```server `gethostip -d fetch15` offline minpoll 8``` to /etc/chrony/chrony.conf, restart chronyd by `sudo /etc/init.d/chrony restart` and wait for few seconds, if you get
```
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

The solution we take now (2016/11/01) is using wired connection as static IP.
By doing so, DNS holds only one IP adress (for wireless connection) for fetch hostname.

### Case description
If you see the following result, it is OK.
```
$ nslookup fetch15.jsk.imi.i.u-tokyo.ac.jp
Server:         127.0.1.1
Address:        127.0.1.1#53

Name:           fetch15.jsk.imi.i.u-tokyo.ac.jp
Address: 133.11.216.145
```

If two or more IP addresses apper, something is wrong.
Please connect display, open a window of network manager, and check that wired connection uses static IP.

### Access point
Define access point setting, such as ssid:
```
/etc/wpa_supplicant/wpa_supplicant.conf
```

### Network status log
You can see network status log, such as radio field intensity or data signaling rate:
```
/var/log/wifi.log
```

Logging script is initialized at:
```
/etc/init/jsk-log-wifi.conf
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
1. Click `ADD A ROBOT` button
1. Select `Fetch` at `Robot type`
1. Type `fetch15` at `Robot name`
1. Type `ws://{fetch's IP adress}:9090/` at `Robot URI`
1. Click `ADD ROBOT` button

### Execute demos
1. Click `fetch15` at `Select Robot` window
1. Select task which are shown with icons.

![select_app](https://user-images.githubusercontent.com/19769486/40872010-7d21d2bc-6681-11e8-8c0b-621f199638dd.png)
