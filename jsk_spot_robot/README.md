jsk_spot_robot
==============

## Manuals

- [Supported Documents of Boston Dynamics](https://www.bostondynamics.com/spot/training/documentation)
- [Spot ROS User Documentation](http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/ros_usage.html#taking-control-of-the-robot)

## Installation

To setup a new internal PC and spot user, Please see [this page](./SetupInternalPCAndSpotUser.md).

### How to set up a catkin workspace for a remote PC

Create a workspace

```bash
source /opt/ros/melodic/setup.bash
mkdir ~/spot_ws/src -p
cd ~/spot_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/k-okada/jsk_robot.git --git -v spot_arm
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_user.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
cd $HOME/spot_ws
catkin init
catkin build -j4 jsk_spot_startup spoteus
```

## How to run

### Bringup spot

First, please turn on spot and turn on motors according to [the OPERATION section of spot user guide](https://www.bostondynamics.com/sites/default/files/inline-files/spot-user-guide.pdf) and power on the internal PC.

Basically, ros systemd services will start automatically. So you can use spot now.


#### superviosur

URI: http://<robot_ip>:9001

#### rwt_app_chooser

URI: http://<robot_ip>:8000/rwt_app_chooser

### Development with a remote PC

Please create `spot_ws` to your PC.

First, connect your development PC's wifi adapter to Access point of the robot.

And for every terminals in this section, Set your ROS_IP and ROS_MASTER_URI and source spot_ws.

```
rossetmaster <robot ip address or robot_name.local>
rossetip
source ~/spot_ws/devel/setup.bash
```

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
(send *ri* :undock)   ;; undock robot
(send *spot* :reset-pose)  ;; change robot pose, use (objects (list *spot*)) to visuailze
(send *ri* :angle-vector (send *spot* :angle-vector) 2000 :default-controller)  ;; send to real robot
(send *ri* :stow-arm)  ;; contorl onbody-api
(send *ri* :sit)
```

### How to set up a catkin workspace in on-bodyPC

First create user account into internal PC
```
ssh spotcore -l spot
sudo adduser k-okada
sudo adduser k-okada spot
sudo adduser k-okada sudo
```

Create a workspace. Make sure that you need to login to spotcore with your account

```bash
ssh spotcore -l k-okada
source /opt/ros/melodic/setup.bash
source ~spot/spot_driver_ws/devel/setup.bash
mkdir ~/spot_ws/src -p
cd ~/spot_ws/src
cd $HOME/spot_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build -j4 jsk_spot_startup spoteus
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/spot_ws/devel/setup.bash" >> ~/.bashrc
```

#### Run apps

You can run apps manually, this is good for debugging your applications.
```bash
roscd jsk_spot_startup/apps/head_lead_demo                                                                       roslaunch head_lead_demo.xml
```

#### Install apps
If you would like to call your apps from rwt_app_chooser, you can 
