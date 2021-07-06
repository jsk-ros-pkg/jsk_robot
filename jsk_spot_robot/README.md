jsk_spot_robot
==============

Currently, this packages require

- [spot_ros]() with [this patch](https://github.com/clearpathrobotics/spot_ros/pull/25)
- [common_msgs]() with [this patch](https://github.com/ros/common_msgs/pull/171)
- [jsk_recognition]() with [this patch](https://github.com/jsk-ros-pkg/jsk_recognition/pull/2579) and [this patch](https://github.com/jsk-ros-pkg/jsk_recognition/pull/2581)

## Manuals

- [Supported Documents of Boston Dynamics](https://www.bostondynamics.com/spot/training/documentation)
- [Spot ROS User Documentation](http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/ros_usage.html#taking-control-of-the-robot)

## How to run

### Setting up udev file for peripheral devices

Create udev rule for spot spinal

```bash
sudo sh -c 'echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", ATTRS{serial}==\"A7044PJ7\", SYMLINK+=\"spot-spinal\", GROUP=\"dialout\"" > /etc/udev/rules.d/99-spot-spinal.rules'
```

and your user to dialout group

```bash
sudo groupadd <your user> dialout
```

### Setting up a catkin workspace for a new user in the internal pc

#### setup a catkin workspace for spot driver

Create a workspace for spot driver

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir $HOME/catkin_ws/src -p
cd $HOME/catkin_ws/src
wstool init .
wstool merge -t . https://github.com/sktometometo/jsk_robot/raw/develop/spot/jsk_spot_robot/jsk_spot.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd $HOME/catkin_ws
catkin init
catkin build -j4 -c
```

After this, please modify the credential file and remove it from git tracking.

```bash
roscd jsk_spot_startup
# modify auth/credential_config.yaml
git update-index --skip-worktree auth/spot_credential.yaml
```

#### setup a catkin workspace for coral usb

Please see [this page](https://github.com/knorth55/coral_usb_ros) for details.

First, install requirements.

```bash
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install python3-opencv
sudo apt-get install ros-melodic-catkin
```

And create a workspace for coral_usb_ros

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir $HOME/coral_ws/src -p
cd $HOME/coral_ws/src
git clone https://github.com/knorth55/coral_usb_ros.git
wstool init .
wstool merge -t . https://github.com/sktometometo/jsk_robot/raw/develop/spot/jsk_spot_robot/jsk_spot_coral.rosinstall
wstool merge -t . coral_usb_ros/fc.rosinstall
wstool merge -t . coral_usb_ros/fc.rosinstall.melodic
wstool update
rosdep install -y -r --from-paths . --ignore-src
cd $HOME/coral_ws
catkin init
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build jsk_spot_startup coral_usb -j4 -c
```

### Bringup spot

First, please turn on spot and turn on motors according to [the OPERATION section of spot user guide](https://www.bostondynamics.com/sites/default/files/inline-files/spot-user-guide.pdf)

After that, please run the ros driver and other basic programs with `jsk_spot_bringup.launch`. You can now control spot from ROS!

```bash
source $HOME/catkin_ws/devel/setup.bash
roslaunch jsk_spot_startup jsk_spot_bringup.launch
```

This launch includes
- driver launch file for spot
- bringup launch for additional peripheral devices (Respeaker, insta 360 air and ublox GPS module)
- teleoperation launch
- interaction launch with Speech-To-Text and Text-To-Speech

And you can run object detection with

```bash
source $HOME/coral_ws/devel/setup.bash
roslaunch jsk_spot_startup object_detection_and_tracking.launch
```

For visualization, you can run RViz with jsk configuration.

```bash
source $HOME/catkin_ws/devel/setup.bash
roslaunch jsk_spot_startup rviz.launch
```

You can control spot with DualShock3 controller. Please see [jsk_spot_teleop](./jsk_spot_teleop/README.md) for more details.

For development, `rosbag_record.launch` and `rosbag_play.launch` are useful for rosbag recording and playing.

```bash
source $HOME/catkin_ws/devel/setup.bash
# Record a rosbag file
roslaunch jsk_spot_startup rosbag_record.launch rosbag:=<absolute file path to rosbag file>
# Play a rosbag file
roslaunch jsk_spot_startup rosbag_play.launch rosbag:=<absolute file path to rosbag file>
```
