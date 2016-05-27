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
catkin build
source devel/setup.bash
```
% Make sure that you have already installed the ``Python NAOqi SDK`` in your computer. If not, you can download it from [here](https://community.aldebaran.com/en/resources/software). After downloading the file, unzip and rename it to ``pynaoqi``, then put it under your home folder. And then, please add python path to your ``.bashrc`` like ``export PYTHONPATH=$HOME/pynaoqi/<your naoqi sdk version>:$PYTHONPATH``. 
 
You need to set NAO_IP and ROS_IP environment variable to launch `jsk_pepper_startup.launch`
```
source ~/catkin_ws/devel/setup.bash
export NAO_IP="olive.jsk.imi.i.u-tokyo.ac.jp" % OR IP address like "133.11.216.xxx"
export ROS_IP="133.11.216.yyy" % OR run rossetip command to set ROS_IP
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
roslaunch nao_apps speech.launch nao_ip:=YOUR_PEPPER_IP
roslaunch nao_interaction_launchers nao_vision_interface.launch nao_ip:=YOUR_PEPPER_IP
roslaunch nao_apps behaviors.launch nao_ip:=YOUR_PEPPER_IP
rosrun jsk_pepper_startup sample.l
$ (demo1) ;; Pepper may speak twice. (This will be fixed as soon as possible.)
```

for developers
--------------
add following source code for debugging.
```
cd  catkin_ws/src
wstool set pepper_robot --git pepper_robot http://github.com/ros-naoqi/pepper_robot
```

naoeus
======

how to make nao model on euslisp
--------------------------------

Install nao mesh files from deb with manual approval of license
```
sudo apt-get install ros-<ros version>-nao-meshes 
catkin build
```