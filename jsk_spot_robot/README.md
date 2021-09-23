jsk_spot_robot
==============

## Manuals

- [Supported Documents of Boston Dynamics](https://www.bostondynamics.com/spot/training/documentation)
- [Spot ROS User Documentation](http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/ros_usage.html#taking-control-of-the-robot)

## How to run

### How to set up catkin workspace (for a user)

Create a workspace

```bash
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install ros-melodic-catkin
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir $HOME/spot_ws/src -p
cd $HOME/spot_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_user.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd $HOME/spot_ws
catkin init
catkin build -j4 -c
```

If you want to use switchbot_ros with spot_basic_behaviors, please add switch_bot token.

```
roscd spot_basic_behaviors
# modify config/switchbot_ros/token.yaml
git update-index --skip-worktree config/switchbot_ros/token.yaml
```

### How to set up spot user in the internal pc.

Spot user have 3 workspaces

- `spot_driver_ws`: a workspace to run driver.launch. which requires python3 build version of geometry3
- `spot_coral_ws`: a workspace to run object_detection.launch ( which includes coral_usb_ros node ) which requires python3 build version of opencv_brindge
- `spot_ws`: a workspace to run other launch ( python2 )

#### setting up `spot_driver_ws`

Create a workspace for spot_driver.

```bash
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install ros-melodic-catkin
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir $HOME/spot_driver_ws/src -p
cd $HOME/spot_driver_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd $HOME/spot_driver_ws
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

#### setting up `spot_coral_ws`

First, follow [Edge TPU dependencies installation section of coral_usb_ros](https://github.com/knorth55/coral_usb_ros#edge-tpu-dependencies-installation)

Create a workspace for coral_usb.

```bash
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install ros-melodic-catkin
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir $HOME/spot_coral_ws/src -p
cd $HOME/spot_coral_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool set coral_usb_ros https://github.com/knorth55/coral_usb_ros.git --git
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_coral.rosinstall
wstool merge -t . coral_usb_ros/fc.rosinstall.melodic
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
cd $HOME/spot_coral_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build -j4 -c
```

After that, please download models for coral_usb_ros.

```
source /opt/ros/$ROS_DISTRO/setup.bash
source $HOME/spot_coral_ws/devel/setup.bash
rosrun coral_usb download_models.py
```

#### setting up `spot_ws`

Create `spot_ws`

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir $HOME/spot_ws/src -p
cd $HOME/spot_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_user.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd $HOME/spot_ws
catkin init
catkin build -j4 -c
```

If you want to use switchbot_ros with spot_basic_behaviors, please add switch_bot token.

```
roscd spot_basic_behaviors
# modify config/switchbot_ros/token.yaml
git update-index --skip-worktree config/switchbot_ros/token.yaml
```

#### Set environmental variables

Set your ROS_IP to WiFi AP address.

```
echo "# ROS_IP
rossetip 10.42.0.1 # change IP address if your wifi AP address is not this." >> ~/.bashrc
```

Add environmental variables for [dialogflow_task_executive](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/dialogflow_task_executive) and [gdrive_ros](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/gdrive_ros)
Please see them for more details.

```
echo "# Credentials
export GOOGLE_APPLICATION_CREDENTIALS=/path/to/service_account_json_file
export DIALOGFLOW_PROJECT_ID=<your dialogflow project id>
export GOOGLE_DRIVE_SETTINGS_YAML=/path/to/pyDrive_setting_yaml" >> ~/.bashrc
```

#### Setting up udev file and add the user to groups

Create udev rule for spot spinal if your pc doesn't have it.

```bash
sudo sh -c 'echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", ATTRS{serial}==\"A7044PJ7\", SYMLINK+=\"spot-spinal\", GROUP=\"dialout\"" > /etc/udev/rules.d/99-spot-spinal.rules'
```

Create udev rule for joy.

TODO

Add spot user to groups

```bash
sudo gpasswd -a <your user> dialout
sudo gpasswd -a <your user> audio
sudo gpasswd -a <your user> plugdev
sudo gpasswd -a <your user> video
```

### Bringup spot

First, please turn on spot and turn on motors according to [the OPERATION section of spot user guide](https://www.bostondynamics.com/sites/default/files/inline-files/spot-user-guide.pdf)

After that, login with spot user and run the driver, object_detection and other basic programs.

#### for spot_driver

```bash
source $HOME/spot_driver_ws/devel/setup.bash
roslaunch jsk_spot_startup driver.launch
```

#### for object detection

```bash
source $HOME/spot_coral_ws/devel/setup.bash
roslaunch jsk_spot_startup object_detection.launch
```

#### for other programs

```
source $HOME/spot_ws/deve/setup.bash
roslaunch jsk_spot_startup jsk_spot_bringup.launch use_driver:=false use_object_detection:=false
```

This launch includes
- bringup launch for additional peripheral devices (Respeaker, insta 360 air and ublox GPS module)
- teleoperation launch
- interaction launch with Speech-To-Text and Text-To-Speech
- so on.

### Development with remote PC

Please create `spot_ws` to your PC.

First, connect your development PC's wifi adapter to Access point of the robot.

And for every terminals in this section, Set your ROS_IP and ROS_MASTER_URI and source spot_ws.

```
rossetip
rossetmaster spot-jsk.local
source ~/spot_ws/devel/setup.bash
```

#### Visualization

For visualization, you can run RViz with jsk configuration.

```bash
source $HOME/spot_ws/devel/setup.bash
roslaunch jsk_spot_startup rviz.launch
```

#### Teleoperation

You can control spot with DualShock3 controller. Please see [jsk_spot_teleop](./jsk_spot_teleop/README.md) for more details.

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
send *ri* :go-pose 1 0
```

And there are some examples in spoteus_demo.

- `sample_basics.l`
  + example of basic usage of Spot
- `sample_navigate_to.l`
  + example of autowalk client
- `sample_visualization.l`
