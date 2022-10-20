# How to setup a internal PC and spot user

This page describes how to setup a internal pc and spot user.

## How to setup a PC

### Install Ubuntu and ROS

TODO

### Setup wifi interfaces

Use [AKEIE](https://www.amazon.co.jp/gp/product/B08NB64TMH/ref=ppx_yo_dt_b_asin_title_o03_s01?ie=UTF8&psc=1) and (Calm USB L Cable (Up side))[https://www.amazon.co.jp/gp/product/B094DGRC9Q/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&th=1]
```
$ lsusb
Bus 001 Device 003: ID 0bda:b812 Realtek Semiconductor Corp.
```

```
$ git clone https://github.com/cilynx/rtl88x2bu.git
cd rtl88x2bu
./deply.sh
sudo nmtui  # to configure network
```

### Setup Joystick

[Push PS and (left-top small) create button to enter pairing mode](https://www.playstation.com/en-us/support/hardware/pair-dualsense-controller-bluetooth/#blue)
```
$ hcitool dev
Devices:
        hci0    00:1B:DC:0D:D0:AD
$ hcitool -i hci0 scan
Scanning ...
        D0:BC:C1:CB:48:37       Wireless Controller
$ sudo bluetoothctl
[bluetooth]# pair D0:BC:C1:CB:48:37
[bluetooth]# trust D0:BC:C1:CB:48:37
[bluetooth]# connect D0:BC:C1:CB:48:37
```

## How to set up the spot user

First, create your user account to internal PC.

```
ssh spot-BD-xxxxxxxx-p.jsk.imi.i.u-tokyo.ac.jp -l spot -p 20022
sudo adduser k-okada
sudo adduser k-okada spot

```

If you are the first user to use spotcore, add `spot` user to sudo group. If someone already using the spotcore, you can skip this section

```
sudo gpasswd -a spot sudo
```

To install systemd service, run following commands. Note that this script start launch file of user's workspace, so usually we expect to run from spot users.
```
rosrun robot_upstart install --provider supervisor --supervisor-priority 10 --roscore
rosrun robot_upstart install --provider supervisor --supervisor-priority 300 --symlink --wait --job jsk_spot_startup jsk_spot_startup/launch/jsk_spot_bringup.launch credential_config:=$(rospack find jsk_spot_startup)/auth/spot_credential.yaml
```

To check output of roslaunch output, please try
```
sudo supervisorctl tail -f jsk_spot_startup stdout
sudo supervisorctl tail -f jsk_spot_startup stderr
``

You can connect to the supervisor console from https://spotcore.jsk.imi.i.u-tokyo.ac.jp:9001/

systemd services of JSK Spot system use workspaces in `spot` user.

- `spot_driver_ws`: a workspace to run driver.launch. which requires python3 build version of geometry3

### Prerequities

Install necessary packages for workspace building

```
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install ros-melodic-catkin
```


### setting up `spot_driver_ws`

Create a workspace for `spot_driver`.

```bash
source /opt/ros/melodic/setup.bash
mkdir ~/spot_driver_ws/src -p
cd ~/spot_driver_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/jsk-ros-pkg/jsk_robot.git --git
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd ~/spot_driver_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build -j4 -c
```

After this, please modify the credential files for spot_driver.

```bash
roscd jsk_spot_startup
# modify auth/credential_config.yaml
git update-index --skip-worktree auth/spot_credential.yaml
```


## Troubleshootig

### Joystick did not respond

Check `Setup Joystick` section and reconnet bluetooth

### Wifi did not connec

Check [roaming aggressiveness configuration](https://github.com/jsk-ros-pkg/jsk_robot/issues/1598)
```
$ wpa_cli get_network 0 bgscan
```

This tells roaming configuation, for example below setting means it scan every 5 seconds when the signal is weak (below -50), and every 3600 seconds otherwise.
```
$ wpa_cli set_network 0 bgscan "\"simple:5:-50:3000\""
```

You can also check the output of wpa_supplicant.
```
$ journalctl -u wpa_supplicant -f
```