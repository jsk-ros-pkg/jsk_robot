jsk_pepper_robot
================

setup environment
-----------------
```
mkdir -p catkin_ws/src
cd  catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_naoqi_robot/pepper.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src
catkin_make
source devel/setup.bash
```
% Make sure that you have already installed the ``Python NAOqi SDK`` in your computer. If not, you can download it from [here](https://community.aldebaran.com/en/resources/software). After downloading the file, unzip and rename it to ``pynaoqi``, then put it under your home folder.
 
You need to set NAO_IP and ROS_IP environment variable to launch `jsk_pepper_startup.launch`
```
source ~/catkin_ws/devel/setup.bash
export NAO_IP="olive.jsk.imi.i.u-tokyo.ac.jp"
export ROS_IP="133.11.216.xxx" % OR run rossetip command to set ROS_IP
```

Install pepper mesh files with manual approval of license
```
sudo apt-get install ros-indigo-pepper-meshes
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

for developers
--------------
add following source code for debugging.
```
cd  catkin_ws/src
wstool set pepper_robot --git pepper_robot http://github.com/ros-naoqi/pepper_robot
```
