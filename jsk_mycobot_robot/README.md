# jsk_mycobot_robot

ROS package for Mycobot ARM robot.

- Mycobot web page
  https://www.elephantrobotics.com/en/mycobot-en/
- GitHub
  https://github.com/elephantrobotics/mycobot_ros

## How to setup the Mycobot robot

under construction 

## How to setup development environment

### My Setup

We need `pymycobot` for conection with M5Stack for the arm. This modules is supported in both python2 and python3

```bash
pip install pymycobot
```

### ROS Environment Setup

Use `wstool`, `rosdep` and `catkin` to checkout and compile the source tree.

```bash
mkdir -p ~/mycobot_ws/src
cd ~/mycobot_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_mycobot_robot/mycobot.rosinstall
wstool update
cd ../
source /opt/ros/melodic/setup.bash
rosdep install -y -r --from-paths src --ignore-src
catkin build jsk_mycobot_startup mycoboteus
source devel/setup.bash
```

## Start ROS Node

```bash
# Mycobot280 without gripper
roslaunch jsk_mycobot_startup mycobot_bringup.launch # rviz:=true for visualize
```

## Use EusLisp model
To control the robot form EusLisp. Please start `roseus` and type as follows.
```

(load "package://mycoboteus/mycobot-interface.l")
;; Mycobot280 without gripper
(mycobot-init)
```

Use `:angle-vector` method to specify the arm joint angle (the max is 100 degree)
```
(send *robot* :angle-vector #f(90.0 15.0 0.0 0.0 0.0 55.0 0.0)) 
```

You can also use `:inverse-kinematics` method to specify the arm pose from target coordinates.
```
(send *ri* :angle-vector (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(280 0 100)) :debug-view t) 3000)
```

To control real robot. you can use *ri* object.
```
(send *ri* :angle-vector (send *robot* :angle-vector) 2000)
```
2000 indicates we ask robot to move for 2000 [msec]

To obtain current robot pose, use `:state :potentio-vector` method.
```
(send *ri* :state :potentio-vector)
```

## Gazebo simulation

Mycobot robot without gripper.
```bash
# Terminal 1
roslaunch mycobot_280_moveit gazebo.launch
# Terminal 2
roscd mycoboteus
roseus mycobot-interface.l
(mycobot-init)
## sample of IK
(send *robot* :rarm :inverse-kinematics (make-coords :pos #f(280 0 100)))
(send *ri* :angle-vector (send *robot* :angle-vector))
```

## Try with moveit

```bash
# Terminal 1
roslaunch jsk_mycobot_startup mycobot_bringup.launch moveit:=true # rviz:=true for visualize
# Or in gazebo:
roslaunch mycobot_280_moveit demo_gazebo.launch
# Terminal 2
roscd mycoboteus
roseus mycobot-interface.l
(mycobot-init)
## try eusIK (goal point) + moveit plan (trajectory)
(send *ri* :angle-vector-motion-plan #f(45 45 45 45 45 45) :move-arm :rarm :total-time 5000)
```



## Trouble shooting

under construction 