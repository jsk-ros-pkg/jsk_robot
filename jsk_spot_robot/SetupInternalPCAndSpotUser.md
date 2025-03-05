# How to setup a internal PC and spot user

This page describes how to setup a internal pc and spot user.

## How to setup a [CORE I/O](https://dev.bostondynamics.com/docs/payload/coreio_documentation)

The CORE I/O is desgined to use filesystem  is read only and user Docekr or Spot Extensions for user applications.


### Setup core-io users

login with default password (ex: ssh -p 20022 10.0.0.3 -el spot)

```
# better to reboot on every command ????
core-io$ passwd ...                       # change password
core-io$ sudo usermod -a -G docker spot   # add spot to docker group
```

# Build Docker containers for Development environment

Software development on core-is uses Docker containers and we provided two containers.

1.  spot_dev_env
2.  `<your username>_dev_env`.

The `spot_dev_env` runs as `spot` user, which will provide basic functions like `roscore` and `jsk_spot_startup` and should be started at boot time.

Users are expected to run `<your username>_dev_env` and create an overay workspace (https://wiki.ros.org/catkin/Tutorials/workspace_overlaying) on top of the workspace of `spot` user.

## Build `spot_dev_env` (for admin)

1. Login to spot as `spot` user or any ararch64 machine as user with uid `1000`. Using an aarch64 machine is recommended because `core-io` has less CPU power.
2. Create a ROS workspace in `~/spot_dev_env`. Then change to the `jsk_spot_robot/coreio/base` directory.
3. Run `make build`. This will build the docker image and compile all packages needed for `jsk_spot_startup`.


If you have built `spot_dev_env` on a machine other than `core-io,

1. Run `docker save spot_dev_env:latest -o spot_dev_env.tar` on your aarch64 machine.
2. Copy it to core-io, e.g. `scp spot_dev_env.tar spot@core-io:/tmp'.
3. Run `docker load -i /tmp/spot_dev_env.tar' on `core-io` machine.
4. As this docker image requires `~/spot_dev_env`. Please set up the ROS workspace under `~/spot_dev_env` as the same as your aarch64 machine.
5. Copy `package.tar` in aarch64 machine to `core-io` under workspace. Log in aarch64 machine and go to `~/spot_dev_env`. Then `scp package.tar spot@core-io:~/spot_dev_env/`.
6. Run `make build` in the `jsk_spot_robot/coreio/base` directory.

## Build `<your username>_dev_env` (for users)

1. Create your ROS workspace on `core-io` as your own userid.
2. go to `jsk_spot_robot/coreio/base` and run `make build`.

# Run Docker container as your development environment

## Run the startup program (for admin)

Login to `core-io` as `spot` user, run `~/start.sh` to start `spot_dev_env` container with tmux to start Bluetooth configuration and jsk_spot_bringup.

- `start.sh` installs `udev` setting and start `start-tmux.sh` through `~/bash.sh`.

```
#!/bin/bash

if [ ! -e /etc/udev/rules.d/88-dualsense.rules ]; then echo 'KERNEL=="js*", ATTRS{name}=="Wireless Controller", SYMLINK+="dualsense"' | sudo tee /etc/udev/rules.d/88-dualsense.rules; fi

~/bash.sh ./start-tmux.sh
```

- `start-tmux.sh` start `connect-bt.sh`, `launch-jsk-spot.sh` and `bash` within `tmux`.

```
#!/bin/bash

tmux set-option -g history-limit 50000 \; new -d -s spot bash \; send "top" SPACE "-c" ENTER\; new-window -d -n bluetooth bash \; next-window \; send "./connect-bt.sh" ENTER \; new-window -d -n roslaunch bash \; next-window \; send "./launch-jsk-spot.sh" ENTER \; next-window \; new-window -d bash \; attach \;

```

- `launch-jsk-spot.sh` starts `jsk_spot_bringup.launch`, for example
```
#!/bin/bash

roslaunch jsk_spot_startup jsk_spot_bringup.launch credential_config:=$(rospack find jsk_spot_startup)/auth/spot_credential.yaml use_voice_text:=true use_gps:=false
```

- `connect-bt.sh` connects to Dualsense joystick.
```
#!/bin/bash -x

while [ 1 ]; do
	if [ -e /dev/input/js0 ]; then sleep 10; continue; fi
	sudo hcitool dev
	sudo hcitool -i hci0 scan

/usr/bin/expect -c '

set MAC "D0:BC:C1:CB:48:37"
set timeout 30

spawn sudo bluetoothctl

send "scan on\n"

expect " Device $MAC " {
    send "trust $MAC\n"
}

expect " Device $MAC " {
    send "pair $MAC\n"
}

expect {
       "AlreadyExists" {
           send "connect $MAC\n"
       }
       "Paired: yes" {
           send "connect $MAC\n"
       }
}

expect "Connection successful" {
    send "quit\n"
}
'
	sleep 3;
done
```

Note:
- This process should be run as upstart or supervisor.
- When `~/bash.sh` is invoked a second time, this existing `spot_dev_env` containers is attached. Therefore, when you exit from this environment, the original `start.sh` container will also be destroyed. To exit from attached environment, use the `[Ctrl-p] [Ctrl-q]`.

## Run your custom development environment  (for users)

1. Login to `core-io` as your local user and run `~/bash.sh` to start your build environment.
2. `~/bash.sh` attach existing container, so use `[Ctrl-p] [Ctrl-q]` to exit from the shell. Otherwise, for example `exit`, it will destroy all container.
3. If you want to create new contaioner, run `NAME=dummy_name ~/bash.sh`
4. In the first time, you don't have an overlay package, so if you run `rospack list | grep $HOME`, you will get nothing. If you have a package to modify, run `catkin <package name>` and `source devel/setup.bash`, you will have your package on your overlay workspace.


# Tips

### Setup development environment

```
core-io$ mkdir ~/spot_dev_env/src -p
core-io$ cd ~/spot_dev_env/src
core-io$ git clone https://github.com/k-okada/jsk_robot.git -b spot_arm
core-io$ cd ~/spot_dev_env/src/jsk_robot/jsk_spot_robot/coreio/base
core-io$ make build bash
```
This procedure create `~/bash.sh`. Please use this script when you start development.

### Setup wifi interfaces

Use [TP-Link AC600 wireless Realtek RTL8811AU Archer T2U Nano](https://www.amazon.co.jp/gp/product/B07MXHJ6KB/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1)
```
$ dmesg
[    7.305827] usb 1-2.1.1: new high-speed USB device number 5 using tegra-xusb
[    7.326404] usb 1-2.1.1: New USB device found, idVendor=2357, idProduct=011e
[    7.326553] usb 1-2.1.1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[    7.326681] usb 1-2.1.1: Product: 802.11ac WLAN Adapter
[    7.326781] usb 1-2.1.1: Manufacturer: Realtek
$ lsusb
Bus 001 Device 005: ID 2357:011e
```

Make sure that the roming configuration works correctly (from core-io)
```
core-io$ sudo wpa_cli get_network 0 bgscan
Selected interface 'wlxac15a246e382'
"simple:30:-80:86400"
```
This scan every 30 seconds, when signal is below -80, and 86400 seconds (24 hours) otherwise.

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


## Settings for SPOT CORE (x86_64)

## wifi settings [Wi-Fi roaming aggressiveness configuration](https://github.com/jsk-ros-pkg/jsk_robot/issues/1598#issuecomment-1247533330)

```
$ git clone https://github.com/cilynx/rtl88x2bu.git
cd rtl88x2bu
./deply.sh
echo 88x2bu | sudo tee /etc/modules-load.d/88x2bu.conf  # to startup on boot time
echo 'install 88x2bu /sbin/modprobe -i 88x2bu && { /sbin/wpa_cli set_network 0 bgscan "\\"simple:5:-50:3000\\"";}' | sudo tee /etc/modprobe.d/88x2bu.conf  # run set_network when load module

sudo nmtui  # to configure network
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
