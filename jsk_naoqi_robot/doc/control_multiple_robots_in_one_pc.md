# Control multiple robots in one PC

## When is this feature useful?

When creating one *ri* instance in one file, and communicate with each other using topics. Currently, we can't create multiple *ri* instances in one file. 

## Requirements

Please use following packages from source:
- jsk_pepper_startup (master branch)
- naoqi_driver [(kochigami-develop branch)](https://github.com/kochigami/naoqi_driver/tree/kochigami-develop)

[Especially, related change is stored in this branch](https://github.com/kochigami/naoqi_driver/tree/add-group-name-to-subscribe-topic-name)  

([allow group-name for subscribing topics](https://github.com/kochigami/naoqi_driver/commit/b752dd8c0559987f77f597d4a0f94894db686000#diff-25d902c24283ab8cfbac54dfa101ad31) + [enable to change the name of boot_config.json](https://github.com/kochigami/naoqi_driver/commit/923310e0520943588cc5bcccaeaea413f1016581#diff-25d902c24283ab8cfbac54dfa101ad31))

```
cd catkin_ws/src
wstool set naoqi_driver --git http://github.com/ros-naoqi/naoqi_driver
wstool update
cd naoqi_driver
git remote add kochigami https://github.com/kochigami/naoqi_driver.git
git fetch kochigami
git checkout -b kochigami-develop kochigami/kochigami-develop

catkin build -c
```

## Sample1 (How topics are covered with group name)

```
roscore
roseus nao-interface.l ; roseus pepper-interface.l
; "robot1" is a group name
nao-init t "robot1" ; pepper-init t "robot1"
```

NAO

```
/dummy_state
/joint_states
/nao_dcm/LeftHand_controller/command
/nao_dcm/RightHand_controller/command
/robot1/animated_speech
/robot1/cmd_vel
/robot1/move_base_simple/goal
/robot1/nao_robot/pose/joint_angles
/robot1/nao_robot/pose/joint_stiffness_trajectory/cancel
/robot1/nao_robot/pose/joint_stiffness_trajectory/feedback
/robot1/nao_robot/pose/joint_stiffness_trajectory/goal
/robot1/nao_robot/pose/joint_stiffness_trajectory/result
/robot1/nao_robot/pose/joint_stiffness_trajectory/status
/robot1/nao_robot/pose/joint_trajectory/cancel
/robot1/nao_robot/pose/joint_trajectory/feedback
/robot1/nao_robot/pose/joint_trajectory/goal
/robot1/nao_robot/pose/joint_trajectory/result
/robot1/nao_robot/pose/joint_trajectory/status
/robot1/speech
/robot_interface_marker_array
/rosout
/rosout_agg
/tf
/tf_static
```

Pepper

```
rostopic list
/dummy_state
/joint_states
/pepper_dcm/LeftHand_controller/command
/pepper_dcm/RightHand_controller/command
/robot1/animated_speech
/robot1/cmd_vel
/robot1/move_base_simple/goal
/robot1/pepper_robot/pose/joint_angles
/robot1/pepper_robot/pose/joint_stiffness_trajectory/cancel
/robot1/pepper_robot/pose/joint_stiffness_trajectory/feedback
/robot1/pepper_robot/pose/joint_stiffness_trajectory/goal
/robot1/pepper_robot/pose/joint_stiffness_trajectory/result
/robot1/pepper_robot/pose/joint_stiffness_trajectory/status
/robot1/pepper_robot/pose/joint_trajectory/cancel
/robot1/pepper_robot/pose/joint_trajectory/feedback
/robot1/pepper_robot/pose/joint_trajectory/goal
/robot1/pepper_robot/pose/joint_trajectory/result
/robot1/pepper_robot/pose/joint_trajectory/status
/robot1/speech
/robot_interface_marker_array
/rosout
/rosout_agg
/tf
/tf_static
```

### Note

Topics which are used in `robot-interface` are not covered with group name.

For example, `/joint_states`, `/tf`, `/tf_static`, `/dummy_state`, `/robot_interface_marker_array`.  

[related issue](https://github.com/jsk-ros-pkg/jsk_robot/issues/1012)


## Sample2 (Two robots speak respectively in one PC)

Robot1: Pepper, Robot2: NAO  
Confirmed with Ubuntu 16.04, ROS kinetic, NAO 2.4.3, Pepper 2.5.5.5

- Put `boot_config.json` with new name under `naoqi_driver/share` as much as the number of robots you want to control. Change `group_name` of `subscribers` group. See examples of [json file1](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/sample/control_multiple_robots/boot_config1.json) and [json file2](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/sample/control_multiple_robots/boot_config2.json).

- Set value of `boot_config_file_name` and `nao_ip` (ex. NAO_IP -> NAO_IP1) like [launch file1](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/sample/control_multiple_robots/sample1.launch) and [launch file2](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/sample/control_multiple_robots/sample2.launch). 

```
roslaunch sample1.launch network_interface:=<network>
roseus sample1.l
; Pepper subscribes `/robot1/speech` and speak
```

```
roslaunch sample2.launch network_interface:=<network>
roseus sample2.l
; NAO subscribes `/robot2/speech` and speak
```

- Pepper will say "hello NAO" and NAO will say "Hello Pepper" respectively in one PC
