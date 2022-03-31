# jsk_unitree_robot

ROS package for Unitree Go1 robot.

## topics

List of topics obtained by following command:
```
pi@raspberrypi:~ $ for topic in $(rostopic list); do echo -e "\n\n\n-\n- $topic\n-"; echo "-- info"; rostopic info $topic; echo "-- hz"; timeout 3 rostopic hz $topic 2> /dev/null; done
```

```
-
- /camera1/point_cloud_face
-
-- info
Type: sensor_msgs/PointCloud2

Publishers:
 * /camera1 (http://192.168.123.13:38961/)

Subscribers: None


-- hz
subscribed to [/camera1/point_cloud_face]
average rate: 16.388
        min: 0.051s max: 0.071s std dev: 0.01025s window: 3



-
- /camera1/range_visual_face
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /camera1 (http://192.168.123.13:38961/)

Subscribers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)


-- hz
subscribed to [/camera1/range_visual_face]
average rate: 15.438
        min: 0.062s max: 0.069s std dev: 0.00302s window: 4



-
- /camera2/point_cloud_chin
-
-- info
Type: sensor_msgs/PointCloud2

Publishers:None

Subscribers:
 * /node_obstacle_traverse (http://raspberrypi:35301/)


-- hz
subscribed to [/camera2/point_cloud_chin]



-
- /camera3/point_cloud_left
-
-- info
Type: sensor_msgs/PointCloud2

Publishers:
 * /camera3 (http://192.168.123.14:34775/)

Subscribers: None


-- hz



-
- /camera3/range_visual_left
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /camera3 (http://192.168.123.14:34775/)

Subscribers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)


-- hz
subscribed to [/camera3/range_visual_left]



-
- /camera4/point_cloud_right
-
-- info
Type: sensor_msgs/PointCloud2

Publishers:
 * /camera4 (http://192.168.123.14:33937/)

Subscribers: None


-- hz
subscribed to [/camera4/point_cloud_right]
average rate: 22.556
        min: 0.034s max: 0.054s std dev: 0.01008s window: 3



-
- /camera4/range_visual_right
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /camera4 (http://192.168.123.14:33937/)

Subscribers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)


-- hz
subscribed to [/camera4/range_visual_right]
average rate: 19.485
        min: 0.022s max: 0.080s std dev: 0.02914s window: 3



-
- /camera5/point_cloud_rearDown
-
-- info
Type: sensor_msgs/PointCloud2

Publishers:
 * /camera5 (http://192.168.123.15
192.168.11.110:39391/)

Subscribers:
 * /node_obstacle_traverse (http://raspberrypi:35301/)


-- hz
subscribed to [/camera5/point_cloud_rearDown]
no new messages



-
- /cmd_odom
-
-- info
Type: nav_msgs/Odometry

Publishers:
 * /ukd_triple_2_goal (http://raspberrypi:44995/)

Subscribers:
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)


-- hz
subscribed to [/cmd_odom]



-
- /cmd_vel
-
-- info
Type: geometry_msgs/Twist

Publishers:None

Subscribers:
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)


-- hz
subscribed to [/cmd_vel]
no new messages



-
- /cmd_vel_2
-
-- info
Type: geometry_msgs/Twist

Publishers:None

Subscribers:
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)


-- hz



-
- /joint_states
-
-- info
Type: sensor_msgs/JointState

Publishers:
 * /node_lcm (http://raspberrypi:34677/)

Subscribers: None


-- hz
subscribed to [/joint_states]
no new messages



-
- /lcm_node/obs_env
-
-- info
Type: sensor_msgs/PointCloud2

Publishers:
 * /node_lcm (http://raspberrypi:34677/)

Subscribers: None


-- hz
subscribed to [/lcm_node/obs_env]
no new messages



-
- /lcm_node/ultrasonic_env
-
-- info
Type: sensor_msgs/PointCloud2

Publishers:
 * /node_lcm (http://raspberrypi:34677/)

Subscribers: None


-- hz
subscribed to [/lcm_node/ultrasonic_env]
no new messages



-
- /move_base_simple/goal
-
-- info
Type: geometry_msgs/PoseStamped

Publishers:None

Subscribers:
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)


-- hz
subscribed to [/move_base_simple/goal]
no new messages



-
- /pointcloud_process/ground_pointcloud
-
-- info
Type: sensor_msgs/PointCloud2

Publishers:
 * /node_obstacle_traverse (http://raspberrypi:35301/)

Subscribers: None


-- hz
subscribed to [/pointcloud_process/ground_pointcloud]
no new messages



-
- /range_front
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)

Subscribers: None


-- hz
subscribed to [/range_front]
average rate: 49.984
        min: 0.019s max: 0.021s std dev: 0.00024s window: 34



-
- /range_left
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)

Subscribers: None


-- hz
subscribed to [/range_left]
average rate: 49.993
        min: 0.020s max: 0.020s std dev: 0.00023s window: 5



-
- /range_right
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)

Subscribers: None


-- hz
subscribed to [/range_right]
average rate: 50.111
        min: 0.019s max: 0.021s std dev: 0.00034s window: 25



-
- /range_ultrasonic_face
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /node_lcm (http://raspberrypi:34677/)

Subscribers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)


-- hz
subscribed to [/range_ultrasonic_face]
average rate: 31.546
        min: 0.012s max: 0.048s std dev: 0.00742s window: 17



-
- /range_ultrasonic_left
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /node_lcm (http://raspberrypi:34677/)

Subscribers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)


-- hz
subscribed to [/range_ultrasonic_left]
average rate: 43.196
        min: 0.011s max: 0.033s std dev: 0.00803s window: 7



-
- /range_ultrasonic_right
-
-- info
Type: sensor_msgs/Range

Publishers:
 * /node_lcm (http://raspberrypi:34677/)

Subscribers:
 * /node_obstacle_avoidance (http://raspberrypi:33969/)


-- hz
subscribed to [/range_ultrasonic_right]
no new messages



-
- /ros2udp/odom
-
-- info
Type: nav_msgs/Odometry

Publishers:
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)

Subscribers:
 * /ukd_triple_2_goal (http://raspberrypi:44995/)


-- hz
subscribed to [/ros2udp/odom]
average rate: 196.643
        min: 0.000s max: 0.041s std dev: 0.01091s window: 25



-
- /ros2udp_motion_mode_adv/joystick
-
-- info
Type: a2_msgs/JoystickA2

Publishers:
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)

Subscribers: None


-- hz



-
- /rosout
-
-- info
Type: rosgraph_msgs/Log

Publishers:
 * /camera4 (http://192.168.123.14:33937/)
 * /camera3 (http://192.168.123.14:34775/)
 * /camera5 (http://192.168.123.15
192.168.11.110:39391/)
 * /camera1 (http://192.168.123.13:38961/)
 * /node_obstacle_traverse (http://raspberrypi:35301/)
 * /node_lcm (http://raspberrypi:34677/)
 * /node_obstacle_avoidance (http://raspberrypi:33969/)
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)
 * /ukd_triple_2_goal (http://raspberrypi:44995/)
 * /ukd_triple_udp_node (http://raspberrypi:45789/)

Subscribers:
 * /rosout (http://raspberrypi:45223/)


-- hz
subscribed to [/rosout]
average rate: 125.469
        min: 0.008s max: 0.008s std dev: 0.00000s window: 2



-
- /rosout_agg
-
-- info
Type: rosgraph_msgs/Log

Publishers:
 * /rosout (http://raspberrypi:45223/)

Subscribers: None


-- hz
subscribed to [/rosout_agg]
no new messages



-
- /tf
-
-- info
Type: tf2_msgs/TFMessage

Publishers:
 * /node_lcm (http://raspberrypi:34677/)
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)
 * /ukd_triple_2_goal (http://raspberrypi:44995/)
 * /ukd_triple_udp_node (http://raspberrypi:45789/)

Subscribers:
 * /node_obstacle_traverse (http://raspberrypi:35301/)
 * /node_lcm (http://raspberrypi:34677/)


-- hz



-
- /tf_static
-
-- info
Type: tf2_msgs/TFMessage

Publishers:None

Subscribers:
 * /node_obstacle_traverse (http://raspberrypi:35301/)
 * /node_lcm (http://raspberrypi:34677/)


-- hz
subscribed to [/tf_static]
no new messages



-
- /ukd_triple/pose
-
-- info
Type: geometry_msgs/PoseStamped

Publishers:
 * /ukd_triple_udp_node (http://raspberrypi:45789/)

Subscribers: None


-- hz
subscribed to [/ukd_triple/pose]
no new messages



-
- /ukd_triple/state
-
-- info
Type: a2_msgs/UKDState

Publishers:
 * /ukd_triple_udp_node (http://raspberrypi:45789/)

Subscribers:
 * /ros2udp_motion_mode_adv (http://raspberrypi:43047/)
 * /ukd_triple_2_goal (http://raspberrypi:44995/)


-- hz



-
- /ukd_triple_2_goal/path_tag_line
-
-- info
Type: nav_msgs/Path

Publishers:
 * /ukd_triple_2_goal (http://raspberrypi:44995/)

Subscribers: None


-- hz
subscribed to [/ukd_triple_2_goal/path_tag_line]



-
- /ukd_triple_2_goal/path_tag_window
-
-- info
Type: nav_msgs/Path

Publishers:
 * /ukd_triple_2_goal (http://raspberrypi:44995/)

Subscribers: None

```
