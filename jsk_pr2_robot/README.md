jsk_pr2_robot
=============


## Setup for Development Users
```
mkdir -p catkin_ws/my_first_demo
cd catkin_ws/my_first_demo
wstool init src
wstool set jsk_demos https://github.com/jsk-ros-pkg/jsk_demos -t src --git
wstool update -t src
source ~applications/ros/hydro/devel/setup.bash
catkin b
```


## Setup for Application Users (for administrator only)

use [pr2.rosinstall](https://github.com/jsk-ros-pkg/jsk_robot/blob/pr2-rosinstall/pr2.rosinstall) to install software

```bash
mkdir -p ros/hydro/src
catkin init

cd src
wstool init
git clone https://github.com/jsk-ros-pkg/jsk_robot.git .jsk_robot_pr2_rosinstall
ln -s .jsk_robot_pr2_rosinstall/pr2.rosinstall .rosinstall
wstool up
cd ..

catkin b
```