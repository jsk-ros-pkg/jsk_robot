jsk_naoqi_robot
===============

JSK original ROS package for NAO and Pepper.
The package name comes from Naoqi OS they use.

Setup Environment
-----------------
% First, you need to install ros. For ros indigo, please refer to install guide like [here](http://wiki.ros.org/indigo/Installation/Ubuntu)

1. Install ``Python NAOqi SDK``
You can download it from [here](https://community.aldebaran.com/en/resources/software). (You may need an account.)  
Please unzip the downloaded file.  
Please create ``pynaoqi`` folder in your home directory.  
Then put the file under your ``pynaoqi`` folder.  

2. Export environment variables in your ``.bashrc``

```
# Python NAOqi SDK version >= 2.5.5
export PYTHONPATH=$HOME/pynaoqi/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages:$PYTHONPATH

# Python NAOqi SDK version < 2.5.5
export PYTHONPATH=$HOME/pynaoqi/<your Python SDK package name>:$PYTHONPATH

export NAO_IP="olive.jsk.imi.i.u-tokyo.ac.jp" % OR IP address like "133.11.216.xxx"
export ROS_IP="133.11.216.yyy" % OR run rossetip command to set ROS_IP
```
% `pose_controller.py` in `naoqi_pose` imports `NaoqiNode` from `naoqi_node.py` in `naoqi_driver_py`.

% `naoqi_node.py` imports `ALProxy` from `naoqi.py`.

% `naoqi.py` is under `pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages/`


% NAO_IP is IP address of Pepper. Pepper tells you their address when pushing their belly button.

% Please install ```ros-indigo-jsk-tools``` to use ```rossetip``` command.


3. Install ROS packages for Pepper

```
mkdir -p catkin_ws/src
cd  catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_naoqi_robot/pepper.rosinstall
wstool update -t src
source /opt/ros/indigo/setup.bash
rosdep install -y -r --from-paths src --ignore-src
sudo apt-get install ros-$ROS_DISTRO-pepper-meshes
sudo apt-get install ros-$ROS_DISTRO-nao-meshes
catkin build
source devel/setup.bash
```

4. (optional) For Pepper developers

Please add following source code for debugging.

```
cd  catkin_ws/src
wstool set pepper_robot --git http://github.com/ros-naoqi/pepper_robot
wstool set naoqi_driver --git http://github.com/ros-naoqi/naoqi_driver
wstool set naoqi_bridge --git http://github.com/ros-naoqi/naoqi_bridge
wstool set naoqi_bridge_msgs --git http://github.com/ros-naoqi/naoqi_bridge_msgs
```

NAO
---

**_jsk_nao_startup_**
  - contains ROS launch files for NAO

[**_naoeus_**](naoeus/README.md)
  - package for controlling NAO via roseus

Pepper
------

[**_jsk_pepper_startup_**](jsk_pepper_startup/README.md)
  - contains ROS launch files for Pepper

[**_peppereus_**](peppereus/README.md)
  - package for controlling Pepper via roseus

[**_jsk_201504_miraikan_**](jsk_201504_miraikan/README.md)
  - demo package which Pepper introduces themselves

