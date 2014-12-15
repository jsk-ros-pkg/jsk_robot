jsk_pepper_robot
================

setup environmnet
-----------------
```
mkdir -p catkin_ws/semi/src
cd  catkin_ws/semi/src
wstool init src
wstool merge https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/jsk_pepper_robot/pepper.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src
catkin_make
source devel/setup.bash
```

running demo
------------
```
rosrun jsk_pepper_startup jsk_pepper_startup.launch
```
```
rosrun jsk_pepper_startup sample.l
$ (demo1)
```
