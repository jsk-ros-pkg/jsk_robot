jsk_nao_startup
==================

What's this?
------------
contains JSK's launch file for startup NAO with ROS

Running startup program
-----------------------

```
rossetip
roslaunch jsk_nao_startup jsk_nao_startup.launch network_interface:=<your network interaface (ex. eth0, enp0s31f6...)>
```

% For network_interface variable, please check `ifconfig` for the interface name your PC uses.

Nodes, topics and services
--------------------------

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
Node [/nao_robot/naoqi_driver]
Publications:
 * /diagnostics [diagnostic_msgs/DiagnosticArray]
 * /joint_states [sensor_msgs/JointState]
 * /nao_robot/naoqi_driver/audio [naoqi_bridge_msgs/AudioBuffer]
 * /nao_robot/naoqi_driver/bumper [naoqi_bridge_msgs/Bumper]
 * /nao_robot/naoqi_driver/camera/bottom/camera_info [sensor_msgs/CameraInfo]
 * /nao_robot/naoqi_driver/camera/bottom/image_raw [sensor_msgs/Image]
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/compressed [sensor_msgs/CompressedImage]
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/theora [theora_image_transport/Packet]
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /nao_robot/naoqi_driver/camera/front/camera_info [sensor_msgs/CameraInfo]
 * /nao_robot/naoqi_driver/camera/front/image_raw [sensor_msgs/Image]
 * /nao_robot/naoqi_driver/camera/front/image_raw/compressed [sensor_msgs/CompressedImage]
 * /nao_robot/naoqi_driver/camera/front/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /nao_robot/naoqi_driver/camera/front/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /nao_robot/naoqi_driver/camera/front/image_raw/theora [theora_image_transport/Packet]
 * /nao_robot/naoqi_driver/camera/front/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /nao_robot/naoqi_driver/camera/front/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /nao_robot/naoqi_driver/chest_touch [naoqi_bridge_msgs/ChestButtonPressed]
 * /nao_robot/naoqi_driver/hand_touch [naoqi_bridge_msgs/HandTouch]
 * /nao_robot/naoqi_driver/head_touch [naoqi_bridge_msgs/HeadTouch]
 * /nao_robot/naoqi_driver/imu/torso [sensor_msgs/Imu]
 * /nao_robot/naoqi_driver/info [naoqi_bridge_msgs/StringStamped]
 * /nao_robot/naoqi_driver/odom [nav_msgs/Odometry]
 * /nao_robot/naoqi_driver/sonar/left [sensor_msgs/Range]
 * /nao_robot/naoqi_driver/sonar/right [sensor_msgs/Range]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

Subscriptions:
 * /cmd_vel [geometry_msgs/Twist]
 * /joint_angles [unknown type]
 * /move_base_simple/goal [unknown type]
 * /speech [std_msgs/String]

Services:
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/compressed/set_parameters
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/compressedDepth/set_parameters
 * /nao_robot/naoqi_driver/camera/bottom/image_raw/theora/set_parameters
 * /nao_robot/naoqi_driver/camera/front/image_raw/compressed/set_parameters
 * /nao_robot/naoqi_driver/camera/front/image_raw/compressedDepth/set_parameters
 * /nao_robot/naoqi_driver/camera/front/image_raw/theora/set_parameters
 * /nao_robot/naoqi_driver/get_loggers
 * /nao_robot/naoqi_driver/set_logger_level
 * /naoqi_driver/fade_leds
 * /naoqi_driver/get_language
 * /naoqi_driver/get_robot_config
 * /naoqi_driver/get_volume
 * /naoqi_driver/play_audio_file
 * /naoqi_driver/reset_leds
 * /naoqi_driver/set_language
 * /naoqi_driver/set_volume		
```

```
Node [/nao_robot/pose/pose_controller]
Publications:
 * /nao_robot/pose/body_pose_naoqi/feedback [naoqi_bridge_msgs/BodyPoseWithSpeedActionFeedback]
 * /nao_robot/pose/body_pose_naoqi/result [naoqi_bridge_msgs/BodyPoseWithSpeedActionResult]
 * /nao_robot/pose/body_pose_naoqi/status [actionlib_msgs/GoalStatusArray]
 * /nao_robot/pose/get_life_state [std_msgs/String]
 * /nao_robot/pose/joint_angles_action/feedback [naoqi_bridge_msgs/JointAnglesWithSpeedActionFeedback]
 * /nao_robot/pose/joint_angles_action/result [naoqi_bridge_msgs/JointAnglesWithSpeedActionResult]
 * /nao_robot/pose/joint_angles_action/status [actionlib_msgs/GoalStatusArray]
 * /nao_robot/pose/joint_stiffness_trajectory/feedback [naoqi_bridge_msgs/JointTrajectoryActionFeedback]
 * /nao_robot/pose/joint_stiffness_trajectory/result [naoqi_bridge_msgs/JointTrajectoryActionResult]
 * /nao_robot/pose/joint_stiffness_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /nao_robot/pose/joint_trajectory/feedback [naoqi_bridge_msgs/JointTrajectoryActionFeedback]
 * /nao_robot/pose/joint_trajectory/result [naoqi_bridge_msgs/JointTrajectoryActionResult]
 * /nao_robot/pose/joint_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /nao_robot/pose/body_pose_naoqi/cancel [unknown type]
 * /nao_robot/pose/body_pose_naoqi/goal [unknown type]
 * /nao_robot/pose/joint_angles [unknown type]
 * /nao_robot/pose/joint_angles_action/cancel [unknown type]
 * /nao_robot/pose/joint_angles_action/goal [unknown type]
 * /nao_robot/pose/joint_stiffness [unknown type]
 * /nao_robot/pose/joint_stiffness_trajectory/cancel [unknown type]
 * /nao_robot/pose/joint_stiffness_trajectory/goal [unknown type]
 * /nao_robot/pose/joint_trajectory/cancel [actionlib_msgs/GoalID]
 * /nao_robot/pose/joint_trajectory/goal [naoqi_bridge_msgs/JointTrajectoryActionGoal]

Services:
 * /nao_robot/pose/body_stiffness/disable
 * /nao_robot/pose/body_stiffness/enable
 * /nao_robot/pose/life/disable
 * /nao_robot/pose/life/enable
 * /nao_robot/pose/life/get_state
 * /nao_robot/pose/pose_controller/get_loggers
 * /nao_robot/pose/pose_controller/set_logger_level
 * /nao_robot/pose/rest
 * /nao_robot/pose/wakeup
```

```
Node [/nao_robot/pose/pose_manager]
Publications:
 * /nao_robot/pose/body_pose/feedback [naoqi_bridge_msgs/BodyPoseActionFeedback]
 * /nao_robot/pose/body_pose/result [naoqi_bridge_msgs/BodyPoseActionResult]
 * /nao_robot/pose/body_pose/status [actionlib_msgs/GoalStatusArray]
 * /nao_robot/pose/joint_trajectory/cancel [actionlib_msgs/GoalID]
 * /nao_robot/pose/joint_trajectory/goal [naoqi_bridge_msgs/JointTrajectoryActionGoal]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /nao_robot/pose/body_pose/cancel [unknown type]
 * /nao_robot/pose/body_pose/goal [unknown type]
 * /nao_robot/pose/joint_trajectory/feedback [naoqi_bridge_msgs/JointTrajectoryActionFeedback]
 * /nao_robot/pose/joint_trajectory/result [naoqi_bridge_msgs/JointTrajectoryActionResult]
 * /nao_robot/pose/joint_trajectory/status [actionlib_msgs/GoalStatusArray]

Services:
 * /nao_robot/pose/pose_manager/get_loggers
 * /nao_robot/pose/pose_manager/set_logger_level 
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
Node [/naoqi_dashboard]
```

```
Node [/speaking_program_is_started_or_terminated]
```