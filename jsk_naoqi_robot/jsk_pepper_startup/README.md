jsk_pepper_startup
==================

What's this?
------------
contains JSK's launch file for startup Pepper with ROS

Running startup program
-----------------------

```
rossetip
roslaunch jsk_pepper_startup jsk_pepper_startup.launch network_interface:=<your network interaface (ex. eth0, enp0s31f6...)>
```

% For network_interface variable, please check `ifconfig` for the interface name your PC uses.  

% In order to confirm if ROS-Pepper is booting, please check with rviz.

```
roscd pepper_bringup/config
rosrun rviz rviz -d pepper.rviz
```

Control Pepper via roseus
-------------------------

Please refer to [README here](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot/peppereus).

Nodes, topics and services
--------------------------

```
Node [/joy_client]
Publications:
 * /pepper_robot/pose/joint_angles [naoqi_bridge_msgs/JointAnglesWithSpeed]
 * /pepper_tweet [std_msgs/String]
 * /rosout [rosgraph_msgs/Log]
 * /speech [std_msgs/String]

Subscriptions:
 * /joy [sensor_msgs/Joy]

Services:
 * /joy_client/get_loggers
 * /joy_client/set_logger_level
```

```
Node [/joy_node]
Publications:
 * /diagnostics [diagnostic_msgs/DiagnosticArray]
 * /joy [sensor_msgs/Joy]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services:
 * /joy_node/get_loggers
 * /joy_node/set_logger_level 
```



```
Node [/naoqi_dashboard]
Publications:
 * /pepper_robot/pose/body_pose_naoqi/cancel [actionlib_msgs/GoalID]
 * /pepper_robot/pose/body_pose_naoqi/goal [naoqi_bridge_msgs/BodyPoseWithSpeedActionGoal]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /diagnostics_agg [diagnostic_msgs/DiagnosticArray]
 * /diagnostics_toplevel_state [diagnostic_msgs/DiagnosticStatus]
 * /pepper_robot/pose/body_pose_naoqi/feedback [naoqi_bridge_msgs/BodyPoseWithSpeedActionFeedback]
 * /pepper_robot/pose/body_pose_naoqi/result [naoqi_bridge_msgs/BodyPoseWithSpeedActionResult]
 * /pepper_robot/pose/body_pose_naoqi/status [actionlib_msgs/GoalStatusArray]
 * /rosout_agg [rosgraph_msgs/Log]

Services:
 * /naoqi_dashboard/get_loggers
 * /naoqi_dashboard/set_logger_level  
```

```
Node [/naoqi_dashboard_aggregator]
Publications:
 * /diagnostics_agg [diagnostic_msgs/DiagnosticArray]
 * /diagnostics_toplevel_state [diagnostic_msgs/DiagnosticStatus]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /diagnostics [diagnostic_msgs/DiagnosticArray]

Services:
 * /diagnostics_agg/add_diagnostics
 * /naoqi_dashboard_aggregator/get_loggers
 * /naoqi_dashboard_aggregator/set_logger_level
```

```
Node [/pepper_robot]
Publications:
 * /diagnostics [diagnostic_msgs/DiagnosticArray]
 * /joint_states [sensor_msgs/JointState]
 * /pepper_robot/audio [naoqi_bridge_msgs/AudioBuffer]
 * /pepper_robot/bumper [naoqi_bridge_msgs/Bumper]
 * /pepper_robot/camera/bottom/camera_info [sensor_msgs/CameraInfo]
 * /pepper_robot/camera/bottom/image_raw [sensor_msgs/Image]
 * /pepper_robot/camera/bottom/image_raw/compressed [sensor_msgs/CompressedImage]
 * /pepper_robot/camera/bottom/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/bottom/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/camera/bottom/image_raw/theora [theora_image_transport/Packet]
 * /pepper_robot/camera/bottom/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/bottom/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/camera/depth/camera_info [sensor_msgs/CameraInfo]
 * /pepper_robot/camera/depth/image_raw [sensor_msgs/Image]
 * /pepper_robot/camera/depth/image_raw/compressed [sensor_msgs/CompressedImage]
 * /pepper_robot/camera/depth/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/depth/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/camera/depth/image_raw/compressedDepth [sensor_msgs/CompressedImage]
 * /pepper_robot/camera/depth/image_raw/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/depth/image_raw/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/camera/depth/image_raw/theora [theora_image_transport/Packet]
 * /pepper_robot/camera/depth/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/depth/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/camera/front/camera_info [sensor_msgs/CameraInfo]
 * /pepper_robot/camera/front/image_raw [sensor_msgs/Image]
 * /pepper_robot/camera/front/image_raw/compressed [sensor_msgs/CompressedImage]
 * /pepper_robot/camera/front/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/front/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/camera/front/image_raw/theora [theora_image_transport/Packet]
 * /pepper_robot/camera/front/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/front/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/camera/ir/camera_info [sensor_msgs/CameraInfo]
 * /pepper_robot/camera/ir/image_raw [sensor_msgs/Image]
 * /pepper_robot/camera/ir/image_raw/compressed [sensor_msgs/CompressedImage]
 * /pepper_robot/camera/ir/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/ir/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/camera/ir/image_raw/theora [theora_image_transport/Packet]
 * /pepper_robot/camera/ir/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /pepper_robot/camera/ir/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /pepper_robot/chest_touch [naoqi_bridge_msgs/ChestButtonPressed]
 * /pepper_robot/hand_touch [naoqi_bridge_msgs/HandTouch]
 * /pepper_robot/head_touch [naoqi_bridge_msgs/HeadTouch]
 * /pepper_robot/imu/base [sensor_msgs/Imu]
 * /pepper_robot/imu/torso [sensor_msgs/Imu]
 * /pepper_robot/info [naoqi_bridge_msgs/StringStamped]
 * /pepper_robot/laser [sensor_msgs/LaserScan]
 * /pepper_robot/odom [nav_msgs/Odometry]
 * /pepper_robot/sonar/back [sensor_msgs/Range]
 * /pepper_robot/sonar/front [sensor_msgs/Range]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

Subscriptions:
 * /cmd_vel [geometry_msgs/Twist]
 * /joint_angles [unknown type]
 * /move_base_simple/goal [unknown type]
 * /speech [std_msgs/String]

Services:
 * /naoqi_driver/fade_leds
 * /naoqi_driver/get_language
 * /naoqi_driver/get_robot_config
 * /naoqi_driver/get_volume
 * /naoqi_driver/play_audio_file
 * /naoqi_driver/reset_leds
 * /naoqi_driver/set_language
 * /naoqi_driver/set_volume
 * /pepper_robot/camera/bottom/image_raw/compressed/set_parameters
 * /pepper_robot/camera/bottom/image_raw/compressedDepth/set_parameters
 * /pepper_robot/camera/bottom/image_raw/theora/set_parameters
 * /pepper_robot/camera/depth/image_raw/compressed/set_parameters
 * /pepper_robot/camera/depth/image_raw/compressedDepth/set_parameters
 * /pepper_robot/camera/depth/image_raw/theora/set_parameters
 * /pepper_robot/camera/front/image_raw/compressed/set_parameters
 * /pepper_robot/camera/front/image_raw/compressedDepth/set_parameters
 * /pepper_robot/camera/front/image_raw/theora/set_parameters
 * /pepper_robot/camera/ir/image_raw/compressed/set_parameters
 * /pepper_robot/camera/ir/image_raw/compressedDepth/set_parameters
 * /pepper_robot/camera/ir/image_raw/theora/set_parameters
 * /pepper_robot/get_loggers
 * /pepper_robot/set_logger_level		      
```

```
Nodes
/pepper_robot/camera/bottom_rectify_color
/pepper_robot/camera/camera_nodelet_manager
/pepper_robot/camera/depth_metric
/pepper_robot/camera/depth_metric_rect
/pepper_robot/camera/depth_rectify_depth
/pepper_robot/camera/depth_registered_hw_metric_rect
/pepper_robot/camera/depth_registered_metric
/pepper_robot/camera/depth_registered_rectify_depth
/pepper_robot/camera/depth_registered_sw_metric_rect
/pepper_robot/camera/front_rectify_color
/pepper_robot/camera/ir_rectify_ir
/pepper_robot/camera/points_xyzrgb_hw_registered
/pepper_robot/camera/points_xyzrgb_sw_registered
/pepper_robot/camera/register_depth_front
```

```
Node [/pepper_robot/pose/pose_controller]
Publications:
 * /pepper_robot/pose/body_pose_naoqi/feedback [naoqi_bridge_msgs/BodyPoseWithSpeedActionFeedback]
 * /pepper_robot/pose/body_pose_naoqi/result [naoqi_bridge_msgs/BodyPoseWithSpeedActionResult]
 * /pepper_robot/pose/body_pose_naoqi/status [actionlib_msgs/GoalStatusArray]
 * /pepper_robot/pose/get_life_state [std_msgs/String]
 * /pepper_robot/pose/joint_angles_action/feedback [naoqi_bridge_msgs/JointAnglesWithSpeedActionFeedback]
 * /pepper_robot/pose/joint_angles_action/result [naoqi_bridge_msgs/JointAnglesWithSpeedActionResult]
 * /pepper_robot/pose/joint_angles_action/status [actionlib_msgs/GoalStatusArray]
 * /pepper_robot/pose/joint_stiffness_trajectory/feedback [naoqi_bridge_msgs/JointTrajectoryActionFeedback]
 * /pepper_robot/pose/joint_stiffness_trajectory/result [naoqi_bridge_msgs/JointTrajectoryActionResult]
 * /pepper_robot/pose/joint_stiffness_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /pepper_robot/pose/joint_trajectory/feedback [naoqi_bridge_msgs/JointTrajectoryActionFeedback]
 * /pepper_robot/pose/joint_trajectory/result [naoqi_bridge_msgs/JointTrajectoryActionResult]
 * /pepper_robot/pose/joint_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /pepper_robot/pose/body_pose_naoqi/cancel [actionlib_msgs/GoalID]
 * /pepper_robot/pose/body_pose_naoqi/goal [naoqi_bridge_msgs/BodyPoseWithSpeedActionGoal]
 * /pepper_robot/pose/joint_angles [naoqi_bridge_msgs/JointAnglesWithSpeed]
 * /pepper_robot/pose/joint_angles_action/cancel [unknown type]
 * /pepper_robot/pose/joint_angles_action/goal [unknown type]
 * /pepper_robot/pose/joint_stiffness [unknown type]
 * /pepper_robot/pose/joint_stiffness_trajectory/cancel [unknown type]
 * /pepper_robot/pose/joint_stiffness_trajectory/goal [unknown type]
 * /pepper_robot/pose/joint_trajectory/cancel [actionlib_msgs/GoalID]
 * /pepper_robot/pose/joint_trajectory/goal [naoqi_bridge_msgs/JointTrajectoryActionGoal]

Services:
 * /pepper_robot/pose/body_stiffness/disable
 * /pepper_robot/pose/body_stiffness/enable
 * /pepper_robot/pose/life/disable
 * /pepper_robot/pose/life/enable
 * /pepper_robot/pose/life/get_state
 * /pepper_robot/pose/pose_controller/get_loggers
 * /pepper_robot/pose/pose_controller/set_logger_level
 * /pepper_robot/pose/rest
 * /pepper_robot/pose/wakeup	 
```

```
Node [/pepper_robot/pose/pose_manager]
Publications:
 * /pepper_robot/pose/body_pose/feedback [naoqi_bridge_msgs/BodyPoseActionFeedback]
 * /pepper_robot/pose/body_pose/result [naoqi_bridge_msgs/BodyPoseActionResult]
 * /pepper_robot/pose/body_pose/status [actionlib_msgs/GoalStatusArray]
 * /pepper_robot/pose/joint_trajectory/cancel [actionlib_msgs/GoalID]
 * /pepper_robot/pose/joint_trajectory/goal [naoqi_bridge_msgs/JointTrajectoryActionGoal]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /pepper_robot/pose/body_pose/cancel [unknown type]
 * /pepper_robot/pose/body_pose/goal [unknown type]
 * /pepper_robot/pose/joint_trajectory/feedback [naoqi_bridge_msgs/JointTrajectoryActionFeedback]
 * /pepper_robot/pose/joint_trajectory/result [naoqi_bridge_msgs/JointTrajectoryActionResult]
 * /pepper_robot/pose/joint_trajectory/status [actionlib_msgs/GoalStatusArray]

Services:
 * /pepper_robot/pose/pose_manager/get_loggers
 * /pepper_robot/pose/pose_manager/set_logger_level 
```

```
Node [/teleop_twist_joy]
Publications:
 * /cmd_vel [geometry_msgs/Twist]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /joy [sensor_msgs/Joy]

Services:
 * /teleop_twist_joy/get_loggers
 * /teleop_twist_joy/set_logger_level 
```

```
Node [/tf_monitor]
Publications:
 * /diagnostics [diagnostic_msgs/DiagnosticArray]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /tf [tf2_msgs/TFMessage]

Services:
 * /tf_monitor/get_loggers
 * /tf_monitor/set_logger_level 
```

```
Node [/speaking_program_is_started_or_terminated]
```

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

Some tips
---------

- If the getting started wizard appears on Pepper's tablet, it may be better to turn it off because some functions are blocked. (ref: [issue 926](https://github.com/jsk-ros-pkg/jsk_robot/issues/926))