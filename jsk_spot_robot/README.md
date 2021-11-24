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

### Setting up a catkin workspace for a new user in the internal pc

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir $HOME/catkin_ws/src -p
cd $HOME/catkin_ws
catkin init
cd src
wstool init .
wstool merge -t . https://github.com/sktometometo/jsk_robot/raw/develop/spot/jsk_spot_robot/jsk_spot.rosinstall
wstool update
rosdep install -y -r --from-paths . --ignore-src
pip3 install -r requirements.txt
catkin build
source $HOME/catkin_ws/devel/setup.bash
```

After this, please modify the credential file and remove it from git tracking.

```bash
roscd jsk_spot_startup
# modify auth/credential_config.yaml
git update-index --skip-worktree auth/spot_credential.yaml
```

### Bringup spot

First, please turn on spot and turn on motors according to [the OPERATION section of spot user guide](https://www.bostondynamics.com/sites/default/files/inline-files/spot-user-guide.pdf)

After that, please run the ros driver and other basic programs with `jsk_spot_bringup.launch`. You can now control spot from ROS!

```bash
roslaunch jsk_spot_startup jsk_spot_bringup.launch
```

This launch includes
- driver launch file for spot
- bringup launch for additional peripheral devices (Respeaker, insta 360 air and ublox GPS module)
- teleoperation launch
- interaction launch with Speech-To-Text and Text-To-Speech

For visualization, you can run RViz with jsk configuration.

```bash
roslaunch jsk_spot_startup rviz.launch
```

You can control spot with DualShock3 controller. Please see [jsk_spot_teleop](./jsk_spot_teleop/README.md) for more details.

For development, `record.launch` and `play.launch` are useful for rosbag recording and playing.

```bash
# Record a rosbag file
roslaunch jsk_spot_startup record.launch rosbag:=<absolute file path to rosbag file>
# Play a rosbag file
roslaunch jsk_spot_startup play.launch rosbag:=<absolute file path to rosbag file>
```
