# jsk_kinova_robot

ROS package for KINOVA Gen3/Gen3 Lite robot.

- KINOVA web page
  https://www.kinovarobotics.com/en/products/gen3-robot
- GitHub
  https://github.com/Kinovarobotics/ros_kortex

## How to setup the network of KINOVA robot

This is an overview of the network setup process.
Refer to the documentation on [KINOVA resouce page](https://www.kinovarobotics.com/en/resources/technical-resources-library) if necessary.

Kinova can be connected with a computer via **USB, Ethernet and Wi-Fi**. 

### Via USB

Micro-B USB to USB type-A cable is required.

1. Connect one end of the micro-B USB to type-A cable to the micro-B USB connector in the robot and connect the other end to the computer.
2. DHCP server on the robot base has worked correctly, and then the computer automatically is assigned an IP address. Try to connect to the robot via _Web App_ below. If not, you have to configure the computer RNDIS settings manually. See the steps below.
3. Open the network settings in your computer and set the IPv4 address:192.168.1.11 and Subnet mask:255.255.255.0. 

### Via Ethernet
USB type-A to Ethernet adapter is required.

1. Connect an Ethernet cable to the robot by using the USB type-A to Ethernet adapter.
2. On your computer, open the network settings and set the IPv4 address:192.168.2.11 and Subnet mask:255.255.255.0.
3. Try to connect to the robot via _Web App_ below.

### Via Wi-Fi
Connecting your computer to ```133.11.xx.xxx``` is necessary.

1. Before you begin, you need to have a wired connection between the computer and the robot. **See the chapter Ethernet or USB and complete it** and see _Open Web App_ below.
2. Open Web App and connect to the robot. 
3. Go to the Wireless & Networks page under the Configurations page. 
4. You can see all of the detected Wi-Fi networks. Choose the networks you use, and fill in the required information.

### Open Web App
You can interact with the arm and perform basic tasks through an Web browser.

1. Confirm connecting the robot with a computer. From the web browser, enter the appropriate IP address for the arm base to access the Web App.
- 192.168.1.10 if connecting via USB
- 192.168.2.10 if connecting via Ethernet
- See the robot base if connecting via Wi-Fi. You can read xx.xx.xx...jp here.
2. Fill in the credentials in the login window and click CONNECT.

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
wstool set jsk-ros-pkg/jsk_robot https://github.com/jsk-ros-pkg/jsk_robot.git --git
wstool update
wstool merge jsk-ros-pkg/jsk_robot/jsk_kinova_robot/jsk_kinova.rosinstall
wstool update
cd ../
source /opt/ros/melodic/setup.bash
rosdep install -y -r --from-paths src --ignore-src
catkin build jsk_kinova_startup kinovaeus
source devel/setup.bash
```

## Start ROS Node

Start real kinova robot. `ip_address` can be given by both ip address and FQDN (e.g. abc.def.jp)  
Use the appropriate ip address according to the type of network you are using  
The IP address is the same as when using Web App

```bash
# Gen3 robot
roslaunch jsk_kinova_startup kinova_bringup.launch ip_address:=xxxx
# Gen3 lite robot
roslaunch jsk_kinova_startup kinova_bringup.launch ip_address:=xxxx arm:=gen3_lite
```

Start rviz from different terminal

```bash
roslaunch jsk_kinova_startup rviz.launch
```

## Use EusLisp model

To control the robot form EusLisp. Please start `roseus` and type as follows.
(Make sure that kinova brings up with `kinova_` prefix, Euslisp model and roseus interface assume that all joint, link and action interface have `kinova_` prefix.)
```

(load "package://kinovaeus/kinova-interface.l")
;; Gen3 with robotiq 2f 85 gripper
(kinova-init :type :gen3_robotiq_2f_85)
;; Gen3 with robotiq 2f 140 gripper
(kinova-init :type :gen3_robotiq_2f_140)
;; Gen3 Lite
(kinova-init :type :gen3_lite_gen3_lite_2f)
```

Use `:angle-vector` method to specify the arm joint angle.
```
(send *kinova* :angle-vector #f(0.0 15.0 180.0 -130.0 0.0 55.0 90.0))
```

You can also use `:inverse-kinematics` method to specify the arm pose from target coordinates.
```
(send *ri* :angle-vector (send *kinova* :arm :inverse-kinematics (make-coords :pos #f(300 0 200) :rpy (float-vector 0 pi/2 0)) :debug-view t) 3000)
```

To move the gripper 50 [mm] up, you can use `move-end-pos` method.
```
(send *kinova* :arm :move-end-pos #f(0 0 -50))
```

You can also use move-end-rot method to turn the gripper.
```
(send *kinova* :arm :move-end-rot -90 :z)
```

To open/close gripper, you can use `start-grasp` and `stop-grasp` method.
```
(send *ri* :start-grasp)
(send *ri* :stop-grasp)
```

To control real robot. you can use *ri* object.
```
(send *ri* :angle-vector (send *kinova* :angle-vector) 2000)
```
2000 indicates we ask robot to move for 2000 [msec]


To obtain current robot pose, use `:state :potentio-vector` method.

```
(send *ri* :state :potentio-vector)
```

To open and close the gripper, You can use `:start-grasp` and `:stop-grasp`.

```
(send *ri* :stop-grasp)
(send *ri* :start-grasp)
```

## Gazebo simulation
Kinova Gen3 robot with robotiq 2f 85 gripper.
```bash
# Terminal 1
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3 gripper:=robotiq_2f_85 robot_name:=arm_gen3 prefix:=kinova_
# Terminal 2
roscd kinovaeus
roseus kinova-interface.l
(kinova-init :type :gen3_robotiq_2f_85)
```

Kinova Gen3 Lite robot with lite 2f gripper.
```bash
# Terminal 1
roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite gripper:=gen3_lite_2f robot_name:=arm_gen3 prefix:=kinova_
# Terminal 2
roscd kinovaeus
roseus kinova-interface.l
(kinova-init :type :gen3_lite_gen3_lite_2f)
```

## Trouble shooting
- The robot doesn't work without errors in roseus and you see the following error in the terminal where you launched the launch file. TODO
  - Turn off the power
  - Make a posture that does not exceed the upper or lower limit of the joint angle
  - Turn it back on
```bash
-----------------------------
Error #38
Type : TRAJECTORY_ERROR_TYPE_JOINT_POSITION_LIMIT
Identifier : TRAJECTORY_ERROR_IDENTIFIER_POSITION
Actuator : 3
Erroneous value is -150.149 but minimum permitted is -150.115 and maximum permitted is 150.115
Additional message is : Invalid Position - Position of actuator(2) in Trajectory Point (37) exceeds limits
-----------------------------
```
This error message above also can be seen in the topic `/arm_gen3/gen3_lite_joint_trajectory_controller/follow_joint_trajectory/result`. 
- Joint limits are different from the ones in the URDF! This causes the problem above.
  - See User guides(ex.[PDF:Gen3_Lite](https://artifactory.kinovaapps.com/artifactory/generic-documentation-public/Documentation/Gen3%20lite/Technical%20documentation/User%20Guide/Gen3_lite_USER_GUIDE_R03.pdf) pp.71-72) 
  - See URDF descriptions([gen3_7dof](https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/arms/gen3/7dof/urdf/gen3_macro.xacro), [gen3_6dof](https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/arms/gen3/6dof/urdf/gen3_macro.xacro), [gen3_lite](https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro.xacro))

- Error in the initial movement in roseus (under investigation)
  - When you used the robot by Web App and then tried to use it by roseus, it occured.
```bash
[ERROR] [1623300893.102460026]: Trajectory has been aborted.
[ERROR] [1623300893.102550198]: Trajectory execution failed in the arm with sub error code 69
The starting point for the trajectory did not match the actual commanded joint angles.
```
---
If you have any question, please feel free to file open at https://github.com/jsk-ros-pkg/jsk_robot/issues
