jsk_pepper_startup
==================

What's this?
------------
contains JSK's launch file for startup Pepper with ROS

Setup environment
-----------------

% First, you need to install ros. For ros indigo, please refer to install guide like [here](http://wiki.ros.org/indigo/Installation/Ubuntu)

1. Install ``Python NAOqi SDK``
You can download it from [here](https://community.aldebaran.com/en/resources/software).
Please unzip the downloaded file.
Please create ``pynaoqi`` folder in your home directory.
Then put the file under your ``pynaoqi`` folder.

2. Export environment variables in your ``.bashrc``

```
export PYTHONPATH=$HOME/pynaoqi/<your Python SDK package name>:$PYTHONPATH
export NAO_IP="olive.jsk.imi.i.u-tokyo.ac.jp" % OR IP address like "133.11.216.xxx"
export ROS_IP="133.11.216.yyy" % OR run rossetip command to set ROS_IP
```

% NAO_IP is IP address of Pepper. Pepper tells you their address when pushing their belly button. 

3. Install ROS packages for Pepper

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
% In addition, please install ```ros-indigo-jsk-tools``` to use ```rossetip``` command.
Otherwise, please write the line below in your ```.bashrc```.

```
export ROS_IP="133.11.216.yyy"
```

4. (optional) For developers

Please add following source code for debugging.

```
cd  catkin_ws/src
wstool set pepper_robot --git http://github.com/ros-naoqi/pepper_robot
```

Running startup program
-----------------------

```
rossetip
roslaunch jsk_pepper_startup jsk_pepper_startup.launch network_interface:=<your network interaface (ex. eth0)>
```

% Temporarly, you have to refer to [this PR](https://github.com/ros-naoqi/pepper_robot/pull/40) in order to execute ```pepper_full.launch```.

% In order to confirm if ROS-Pepper is booting, please check with rviz.

```
roscd pepper_bringup/config
rosrun rviz rviz -d pepper.rviz
```

Control Pepper via roseus
-------------------------

Please refer to [README here](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot/peppereus).


Sample demo
-----------

```
roslaunch nao_apps speech.launch nao_ip:=YOUR_PEPPER_IP
roslaunch nao_interaction_launchers nao_vision_interface.launch nao_ip:=YOUR_PEPPER_IP
roslaunch nao_apps behaviors.launch nao_ip:=YOUR_PEPPER_IP
rosrun jsk_pepper_startup sample.l
$ (demo1) ;; Pepper may speak twice. (This will be fixed as soon as possible.)
```

If you failed in launching jsk_pepper_startup.launch
----------------------------------------------------

1. Please try deleting all the terminals you created before, and even rebooting your PC.
If another terminal of ```roscore``` has been left and the connected network has changed recently, connecting your PC and pepper may fail.

2. If your terminal log looks like below, please reboot pepper.

```
front_cameraCamera Handle is empty - cannot retrieve image
front_cameraMight be a NAOqi problem. Try to restart the ALVideoDevice.
```