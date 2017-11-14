jsk_pepper_robot
================

setup environment
-----------------
% First, you need to install ros. For ros indigo, please refer to install guide like [here](http://wiki.ros.org/indigo/Installation/Ubuntu)

```
mkdir -p catkin_ws/src
cd  catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_naoqi_robot/pepper.rosinstall
wstool update -t src
source /opt/ros/indigo/setup.bash
rosdep install -y -r --from-paths src --ignore-src
catkin build
source devel/setup.bash
```
% Make sure that you have already installed the ``Python NAOqi SDK`` in your computer. If not, you can download it from [here](https://community.aldebaran.com/en/resources/software). Be sure to sign in and register a developer program. After downloading the file, unzip and rename it to ``pynaoqi``, then put it under your home folder. And then, please add python path to your ``.bashrc`` like ``export PYTHONPATH=$HOME/pynaoqi/<your naoqi sdk version>:$PYTHONPATH``. 
 
You need to set NAO_IP and ROS_IP environment variable to launch `jsk_pepper_startup.launch`
```
source ~/catkin_ws/devel/setup.bash
export NAO_IP="olive.jsk.imi.i.u-tokyo.ac.jp" % OR IP address like "133.11.216.xxx"
export ROS_IP="133.11.216.yyy" % OR run rossetip command to set ROS_IP
```

% Temporary caution
You have to refer to [this PR](https://github.com/ros-naoqi/pepper_robot/pull/40) in order to execute ```pepper_full.launch```.

Install pepper mesh files with manual approval of license
```
sudo apt-get install ros-indigo-pepper-meshes
```

running demo
------------
```
roslaunch jsk_pepper_startup jsk_pepper_startup.launch network_interface:=<YOUR_NETWORK_INTERFACE (ex. eth0)>
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
wstool set pepper_robot --git http://github.com/ros-naoqi/pepper_robot
```

trouble shooting
----------------
If you failed in launching jsk_pepper_startup.launch,
please check below.

1. Please try deleting all the terminals you created before, and even rebooting your PC.
If another terminal of ```roscore``` has been left and the connected network has changed recently, connecting your PC and pepper may fail.

2. If the log shows below (*), please reboot pepper.
(*)
```
front_cameraCamera Handle is empty - cannot retrieve image
front_cameraMight be a NAOqi problem. Try to restart the ALVideoDevice.
```

peppereus
=========

Here is a list of joints when accessing Pepper.
ex:
```
(send *pepper* :reset-pose)
=> #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)
       0    1    2   3    4     5     6     7    8     9    10   11   12   13  14
```
```
0: :knee-p
1: :hip-r
2: :hip-p
3: :larm :shoulder-p
4: :larm :shoulder-r
5: :larm :elbow-y
6: :larm :elbow-p
7: :larm :wrist-y
8: :rarm :shoulder-p
9: :rarm :shoulder-r
10: :rarm :elbow-y
11: :rarm :elbow-p
12: :rarm :wrist-y
13: :head :neck-y
14: :head :neck-p
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