jsk_spot_robot
==============

## Manuals

- [Supported Documents of Boston Dynamics](https://www.bostondynamics.com/spot/training/documentation)
- [Spot ROS User Documentation](http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/ros_usage.html#taking-control-of-the-robot)

## Installation

To setup a new internal PC and spot user, Please see [this page](./SetupInternalPCAndSpotUser.md).

### How to set up a catkin workspace for a remove PC

Create a workspace

```bash
source /opt/ros/melodic/setup.bash
mkdir ~/spot_ws/src -p
cd ~/spot_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_user.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
cd $HOME/spot_ws
catkin init
catkin build -j4 -c
```

## How to run

### Bringup spot

First, please turn on spot and turn on motors according to [the OPERATION section of spot user guide](https://www.bostondynamics.com/sites/default/files/inline-files/spot-user-guide.pdf) and power on the internal PC.

Basically, ros systemd services will start automatically. So you can use spot now.
Details about ros systemd services of JSK Spot, Please see [System Overview page](./SystemOverview.md).

#### Start basic roslaunch manually

If you want to launch basic ROS launches manually for some reason, please login `spot` user and follow this instruction.

##### for spot_driver

```bash
source $HOME/spot_driver_ws/devel/setup.bash
roslaunch jsk_spot_startup driver.launch
```

##### for object detection

```bash
source $HOME/spot_coral_ws/devel/setup.bash
roslaunch jsk_spot_startup object_detection.launch
```

##### for other programs

```
source $HOME/spot_ws/deve/setup.bash
roslaunch jsk_spot_startup jsk_spot_bringup.launch use_driver:=false use_object_detection:=false
```

This launch includes
- bringup launch for additional peripheral devices (Respeaker, insta 360 air and ublox GPS module)
- teleoperation launch
- interaction launch with Speech-To-Text and Text-To-Speech
- so on.

### Teleoperation

You can control spot with DualSense controller. Please see [jsk_spot_teleop](./jsk_spot_teleop/README.md) for more details.

### Web UI

Spot have some web UI.

#### cockpit

URI: https://<robot_ip>:21443

TODO

#### rwt_app_chooser

TODO

### VNC Server

You can access VNC Server of Spot PC.
Ports are below.

- Belka: 5100
- Strelka: 21000

### Development with a remote PC

Please create `spot_ws` to your PC.

First, connect your development PC's wifi adapter to Access point of the robot.

And for every terminals in this section, Set your ROS_IP and ROS_MASTER_URI and source spot_ws.

```
rossetip
rossetmaster <robot ip address or robot_name.local>
source ~/spot_ws/devel/setup.bash
```

#### Visualization

For visualization, you can run RViz with jsk configuration.

```bash
source $HOME/spot_ws/devel/setup.bash
roslaunch jsk_spot_startup rviz.launch
```

#### rosbag record and play

For development, `rosbag_record.launch` and `rosbag_play.launch` are useful for rosbag recording and playing.

```bash
source $HOME/spot_ws/devel/setup.bash
# Record a rosbag file
roslaunch jsk_spot_startup rosbag_record.launch rosbag:=<absolute file path to rosbag file>
# Play a rosbag file ( don't run this with setting ros_master_uri to the robot )
roslaunch jsk_spot_startup rosbag_play.launch rosbag:=<absolute file path to rosbag file>
```

#### Control API

If you want to control Spot, there are basically 3 options.

- Use spoteus ( roseus client, recommended )
- Use python client ( spot_ros_client, experimental )
- Use raw ROS interface

##### spoteus

spoteus is roseus client for ROS Interface of spot_ros.

Open a terminal, setting up it for spot and run roseus repl. ( emacs shell or euslime are recommended. )

Then initialize it for spot

```
(load "package://spoteus/spot-interface.l")
(spot-init)
```

You can now control spot from roseus.
for example, when you want to move spot 1m forward, type.

```
send *ri* :go-pos 1 0
```

And there are some examples in `spoteus/demo`.

- `sample_basics.l`
  + example of basic usage of Spot
- `sample_navigate_to.l`
  + example of autowalk client
- `sample_visualization.l`
