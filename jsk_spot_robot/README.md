jsk_spot_robot
==============

## Manuals

- [Supported Documents of Boston Dynamics](https://www.bostondynamics.com/spot/training/documentation)
- [Spot ROS User Documentation](http://www.clearpathrobotics.com/assets/guides/melodic/spot-ros/ros_usage.html#taking-control-of-the-robot)

## How to run

### Setting up a catkin workspace for a new user in the internal pc

```
$ mkdir $HOME/catkin_ws/src -p
$ cd $HOME/catkin_ws/src
$ wstool init .
$ wstool merge -t . https://raw.githubusercontent.com/k-okada/jsk_robot/spot/jsk_spot_robot/jsk_spot.rosinstall
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ rosdep install -y -r --from-paths . --ignore-src
$ cd ../
$ catkin build spoteus jsk_spot_startup
$ source $HOME/catkin_ws/devel/setup.bash
```

### Bringup spot

First, please turn on spot and turn on motors according to [the OPERATION section of spot user guide](https://www.bostondynamics.com/sites/default/files/inline-files/spot-user-guide.pdf)

After that, please run the ros driver. You can now control spot from ROS!

```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ source $HOME/catkin_ws/deve/setup.bash
$ roslaunch jsk_spot_startup driver.launch username:=<username> password:=<password>
```

You can run RViz already configured for spot.

```
# in another terminal
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ source $HOME/catkin_ws/deve/setup.bash
$ roslaunch jsk_spot_startup rviz.launch
```
