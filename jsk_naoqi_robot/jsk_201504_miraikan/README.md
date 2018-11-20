jsk_201504_miraikan
===================

What is this?
-------------

Pepper@JSK introduces him/ herself.  
Pepper can speak three languages (Japanese, Chinese and English).  

Requirements
------------

- jsk_naoqi_robot environment: Please follow instructions [here](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/README.md)
- naoqi_driver (kochigami-develop)
- naoqi_bridge_msgs (kochigami-develop)

For the time being, please use `kochigami-develop` branch of `naoqi_driver` and `naoqi_bridge_msgs`.

```
cd  catkin_ws/src
wstool set naoqi_driver --git https://github.com/ros-naoqi/naoqi_driver
wstool set naoqi_bridge_msgs --git https://github.com/ros-naoqi/naoqi_bridge_msgs
wstool update
cd naoqi_driver
git remote add kochigami https://github.com/kochigami/naoqi_driver.git
git fetch kochigami
git checkout -b kochigami-develop kochigami/kochigami-develop


cd ../naoqi_bridge_msgs
git remote add kochigami https://github.com/kochigami/naoqi_bridge_msgs.git
git fetch kochigami
git checkout -b kochigami-develop kochigami/kochigami-develop
```

How to run demo?
----------------

This demo was tested with Pepper (NAOqi version: 2.5.5.5) and ROS kinetic.

- (Optional) Preparation for Chinese demo

Please move mp3 files under `/jsk_201504_miraikan/file` to `/home/nao/audio_file` inside Pepper's PC.  
You can log in to Pepper's PC by `ssh nao@<Pepper's IP>`.  
If there is no `audio_file` folder, please make it under `/home/nao/`.  
You can move mp3 files as follows:

```
roscd jsk_201504_miraikan
cd file
scp <file name>  nao@<Pepper IP>:/home/nao/audio_file/
```

- Please execute these for all languages

```
roslaunch jsk_pepper_startup jsk_pepper_startup.launch
roseus mirai-demo-2015-0413.l
demo ; Japanese
demo :en ; English
demo :chi ; Chinese
```