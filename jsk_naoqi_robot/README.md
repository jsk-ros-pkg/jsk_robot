jsk_pepper_robot
================

setup environment
-----------------
```
mkdir -p catkin_ws/src
cd  catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_pepper_robot/pepper.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src
catkin_make
source devel/setup.bash
```

Make sure that you have NAO_IP environment variable and PYTHON PATH to your NAOqi Python SDK path. So your `.bashrc` would be
```
source ~/catkin_ws/devel/setup.bash
export PYTHONPATH=$HOME/pynaoqi:$PYTHONPATH
export NAO_IP="olive.jsk.imi.i.u-tokyo.ac.jp"
```

running demo
------------
```
roslaunch jsk_pepper_startup jsk_pepper_startup.launch
```
```
rosrun jsk_pepper_startup sample.l
$ (demo1)
```
