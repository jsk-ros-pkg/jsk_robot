# jsk_kinova_robot

ROS package for KINOVA Gen3/Gen3 Lite robot.

- KINOVA web page
  https://www.kinovarobotics.com/en/products/gen3-robot
- GitHub
  https://github.com/Kinovarobotics/ros_kortex


## How to setup development environment

Use `wstool`, `rosdep` and `catkin` to checkout and compile the source tree.

```bash
mkdir -p ~/kinova_ws/src
cd ~/kinova_ws/src
wstool init
### TODO: Change this line 708yamaguchi -> jsk-ros-pkg if approved ##
wstool merge https://raw.githubusercontent.com/708yamaguchi/jsk_robot/kinova-gen3/jsk_kinova_robot/kinova.rosinstall
wstool update
cd ../
source /opt/ros/melodic/setup.bash
rosdep install -y -r --from-paths src --ignore-src
# TODO: add catkin build jsk_kinova_startup
catkin build kinovaeus
source devel/setup.bash
```

## KINOVA Setup

TODO:

Setup real kinova robot

MEMO:

https://github.com/jsk-ros-pkg/jsk_robot/blob/dec36f6906b5cfad0a1debb19b9d17b5e7753e75/jsk_denso_robot/README.md

## Start ROS Node

TODO:

Start real kinova robot

```bash
roslaunch xxx
```

## Use EusLisp model
To control the robot form EusLisp. Please start `roseus` and type as follows.
```
;; Gen3
(load "package://kinovaeus/gen3-interface.l")
(gen3-init)
;; Gen3 Lite
(load "package://kinovaeus/gen3-lite-interface.l")
(gen3-lite-init)
```

Use `:angle-vector` method to specify the arm joint angle.
```
(send *gen3* :angle-vector #f(0.0 15.0 180.0 -130.0 0.0 55.0 90.0))
```

You can also use `:inverse-kinematics` method to specify the arm pose from target coordinates.
```
(send *gen3* :arm :inverse-kinematics (make-coords :pos #f(300 0 200) :rpy (float-vector pi  0 pi)) :debug-view t)
```

To move the gripper 50 [mm] up, you can use `move-end-pos` method.
```
(send *gen3* :arm :move-end-pos #f(0 0 -50))
```

You can also use move-end-rot method to turn the gripper.
```
(send *gen3* :arm :move-end-rot -90 :z)
```

To control real robot. you can use *ri* object.
```
(send *ri* :angle-vector (send *gen3* :angle-vector) 2000)
```
2000 indicates we ask robot to move for 2000 [msec]

To obtain current robot pose, use :state :potentio-vector method.
```
(send *ri* :state :potentio-vector)
```

To open and close the gripper. You can use :start-grasp and :stop-grasp.
```
(send *ri* :stop-grasp)
(send *ri* :start-grasp)
```

## Gazebo simulation
Kinova Gen3 robot with robotiq 2f 85 gripper.
```bash
# Terminal 1
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3 gripper:=robotiq_2f_85
# Terminal 2
roscd kinovaeus
roseus gen3-interface.l
```

Kinova Gen3 Lite robot with lite 2f gripper.
```bash
# Terminal 1
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite gripper:=gen3_lite_2f
# Terminal 2
roscd kinovaeus
roseus gen3-lite-interface.l
```

---
If you have any question, please feel free to file open at https://github.com/jsk-ros-pkg/jsk_robot/issues
