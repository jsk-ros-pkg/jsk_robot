# jsk_kinova_robot

ROS package for KINOVA Gen3/Gen3 Lite robot.

- KINOVA web page
  https://www.kinovarobotics.com/en/products/gen3-robot
- GitHub
  https://github.com/Kinovarobotics/ros_kortex


## How to setup development environment

### Conan Setup

You need to install `conan` (Decentralized, open-source (MIT), C/C++ package manager) to build kinova packages

```bash
sudo apt install python3 python3-pip
python3 -m pip install --user conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
```

### ROS Environment Setup

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
catkin build jsk_kinova_startup kinovaeus
source devel/setup.bash
```

## Start ROS Node

Start real kinova robot

```bash
roslaunch jsk_kinova_startup kinova_bringup.launch
```

Start rviz from different terminal

```bash
roslaunch jsk_kinova_startup rvi.launch
```

## Use EusLisp model
To control the robot form EusLisp. Please start `roseus` and type as follows.
```
;; Gen3
(load "package://kinovaeus/kinova-interface.l")
(kinova-init)
;; Gen3 Lite
(load "package://kinovaeus/kinova-lite-interface.l")
(kinova-init :type :gen3_lite_gen3_lite_2f)

## Use EusLisp model

To control the robot form EusLisp. Please start `roseus` and type as follows.

```
(load "package://kinovaeus/kinova-interface.l")
(kinova-init)
```

Use `:angle-vector` method to specify the arm joint angle.
```
(send *kinova* :angle-vector #f(0.0 15.0 180.0 -130.0 0.0 55.0 90.0))
```

You can also use `:inverse-kinematics` method to specify the arm pose from target coordinates.
```
(send *kinova* :arm :inverse-kinematics (make-coords :pos #f(300 0 200) :rpy (float-vector pi  0 pi)) :debug-view t)
```

To move the gripper 50 [mm] up, you can use `move-end-pos` method.
```
(send *kinova* :arm :move-end-pos #f(0 0 -50))
```

You can also use move-end-rot method to turn the gripper.
```
(send *kinova* :arm :move-end-rot -90 :z)
```

To control real robot. you can use *ri* object.
```
(send *ri* :angle-vector (send *kinova* :angle-vector) 2000)
```
2000 indicates we ask robot to move for 2000 [msec]

To obtain current robot pose, use :state :potentio-vector method.
```
(send *kinova* :arm :move-end-pos #f(0 0 -50))
```

To obtain current robot pose, use `:state :potentio-vector` method.

```
(send *ri* :state :potentio-vector)
```

To open and close the gripper. You can use `:start-grasp` and `:stop-grasp`.

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
roseus kinova-interface.l
(kinova-init :type :gen3_robotiq_2f_85)
```

Kinova Gen3 Lite robot with lite 2f gripper.
```bash
# Terminal 1
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite gripper:=gen3_lite_2f
# Terminal 2
roscd kinovaeus
roseus kinova-interface.l
(kinova-init :type :gen3_lite_gen3_lite_2f)
````

---
If you have any question, please feel free to file open at https://github.com/jsk-ros-pkg/jsk_robot/issues
