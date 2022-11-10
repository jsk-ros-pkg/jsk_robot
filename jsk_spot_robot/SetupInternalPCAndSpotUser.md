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
echo 88x2bu | sudo tee /etc/modules-load.d/88x2bu.conf  # to startup on boot time
echo 'install 88x2bu /sbin/modprobe -i 88x2bu && { /sbin/wpa_cli set_network 0 bgscan "\\"simple:5:-50:3000\\"";}' | sudo tee /etc/modprobe.d/88x2bu.conf  # run set_network when load module

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

### Setup timezone

```
sudo timedatectl set-timezone Asia/Tokyo
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
rosrun robot_upstart install --provider supervisor --supervisor-priority 300 --symlink --wait --job jsk_spot_startup jsk_spot_startup/launch/jsk_spot_bringup.launch credential_config:=$(rospack find jsk_spot_startup)/auth/spot_credential.yaml use_app_manager:=false
rosrun robot_upstart install --provider supervisor --supervisor-priority 400 --symlink --wait --job app_manager jsk_robot_startup/lifelog/app_manager.launch use_applist:=false respawn:=false
```

To check output of roslaunch output, please try
```
sudo supervisorctl tail -f jsk_spot_startup stdout
sudo supervisorctl tail -f jsk_spot_startup stderr
```

You can connect to the supervisor console from https://spotcore.jsk.imi.i.u-tokyo.ac.jp:9001/

systemd services of JSK Spot system use workspaces in `spot` user.

- `spot_driver_ws`: a workspace to run driver.launch. which requires python3 build version of geometry3

### Prerequities

Install necessary packages for workspace building

```
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy python3-opencv python3-sip-dev python3-defusedxml
sudo apt-get install ros-melodic-catkin ros-melodic-vision-msgs
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
catkin build -j4 tf2_ros cv_bridge jsk_spot_startup spoteus robot_upstart
```

After this, please modify the credential files for spot_driver.

```bash
roscd jsk_spot_startup
# modify auth/credential_config.yaml
git update-index --skip-worktree auth/spot_credential.yaml
```

Note that `rosdep install ...` did not install all dependencies, you need to install `python3-` modules manualy. or run `ROS_PYTHON_VERSION=3 rosdep install ...`. But this will install `python3-catkin-pkg` which remove `python2-catkin-pkg` package and all `ros-melodic-*` packages.

Workaround is to install only jsk_robot direcotry. (But you still re-install `ros-melodic-jsk-tools`)
```
ROS_PYTHON_VERSION=3 rosdep install -r --from-paths ~/spot_driver_ws/src/jsk_robot --ignore-src
```
or create dummy package that did not conflict each other.

```
#!/bin/bash

set -x
set -e

TMPDIR=/tmp/tmp-$$

mkdir ${TMPDIR}
cd ${TMPDIR}
apt download python3-catkin-pkg
dpkg-deb -R python3-catkin-pkg_0.5.2-100_all.deb ${TMPDIR}/src-python3-catkin-pkg
rm -fr ${TMPDIR}/src-python3-catkin-pkg/usr/bin/
sed -i /^Conflicts:/d ${TMPDIR}/src-python3-catkin-pkg/DEBIAN/control
sed -i '/^Version:/ s/$/0/' ${TMPDIR}/src-python3-catkin-pkg/DEBIAN/control
cat ${TMPDIR}/src-python3-catkin-pkg/DEBIAN/control
# edit DEBIAN/postinst
dpkg-deb -b ${TMPDIR}/src-python3-catkin-pkg python3-catkin-pkg-dummy.deb

apt download python-catkin-pkg
dpkg-deb -R python-catkin-pkg_0.5.2-100_all.deb ${TMPDIR}/src-python2-catkin-pkg
rm -fr ${TMPDIR}/src-python2-catkin-pkg/usr/bin/
sed -i 's/, python3-catkin-pkg//' ${TMPDIR}/src-python2-catkin-pkg/DEBIAN/control
sed -i '/^Version:/ s/$/0/' ${TMPDIR}/src-python2-catkin-pkg/DEBIAN/control
cat ${TMPDIR}/src-python2-catkin-pkg/DEBIAN/control
# edit DEBIAN/postinst
dpkg-deb -b ${TMPDIR}/src-python2-catkin-pkg python2-catkin-pkg-no-conflict.deb

apt download python3-rosdep
dpkg-deb -R python3-rosdep_0.22.1-1_all.deb ${TMPDIR}/src-python3-rosdep
rm -fr ${TMPDIR}/src-python3-rosdep/usr/bin/
sed -i /^Conflicts:/d ${TMPDIR}/src-python3-rosdep/DEBIAN/control
sed -i '/^Version:/ s/$/0/' ${TMPDIR}/src-python3-rosdep/DEBIAN/control
cat ${TMPDIR}/src-python3-rosdep/DEBIAN/control
# edit DEBIAN/postinst
dpkg-deb -b ${TMPDIR}/src-python3-rosdep python3-rosdep-dummy.deb

apt download python-rosdep
dpkg-deb -R python-rosdep_0.22.1-1_all.deb ${TMPDIR}/src-python-rosdep
rm -fr ${TMPDIR}/src-python-rosdep/usr/bin/
sed -i 's/, python3-rosdep//' ${TMPDIR}/src-python-rosdep/DEBIAN/control
sed -i '/^Version:/ s/$/0/' ${TMPDIR}/src-python-rosdep/DEBIAN/control
cat ${TMPDIR}/src-python-rosdep/DEBIAN/control
# edit DEBIAN/postinst
dpkg-deb -b ${TMPDIR}/src-python-rosdep python-rosdep-no-conflict.deb

set +x
echo "scp ${TMPDIR}/python2-catkin-pkg-no-conflict.deb to your environment"
echo "scp ${TMPDIR}/python3-catkin-pkg-dummy.deb to your environment"
echo "scp ${TMPDIR}/python-rosdep-no-conflict.deb to your environment"
echo "scp ${TMPDIR}/python3-rosdep-dummy.deb to your environment"
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

### rwt_app_chooser did not respond

http://spotcore:8000/rwt_app_chooser/#!task/<robot> did not show any apps, and following command did not returns any apps, make sure that you have run `rosdep update`.

```
$ rosservice call /SpotCORE/list_apps
```
