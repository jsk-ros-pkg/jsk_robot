jsk_pepper_startup
==================

What's this?
------------
contains JSK's launch file for startup Pepper with ROS

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