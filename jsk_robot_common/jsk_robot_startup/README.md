jsk_robot_startup
===

## lifelog

see [lifelog/README.md](lifelog/README.md)

## scripts/ConstantHeightFramePublisher.py
![pointcloud_to_scan_base_tf_squat.png](images/pointcloud_to_scan_base_tf_squat.png)
![pointcloud_to_scan_base_tf_stand.png](images/pointcloud_to_scan_base_tf_stand.png)

This script provides a constant height frame from the ground to get a imagenary laser scan for pointcloud_to_laserscan package.
Biped robots need to use this constant frame to get constant laser scan for 2D SLAM package for wheeled ones like gmapping,
because the pose of biped robots including height of the base link changes during a task in contrast to wheeled ones.
In this frame, x, y and yaw is same as base frame of the robot body, z is constant and roll and pitch is same as the ground.

### Parameters

* `~parent_frame` (String, default: "BODY")

  This parameter indicates the parent frame of the constant height frame, which is expected to be a base frame of the robot body.

* `~odom_frame` (String, default: "odom")

  This parameter indicates the odometry frame on the ground.

* `~frame_name` (String, default: "pointcloud_to_scan_base")

  This parameter indicates the name of the constant frame.

* `~rate` (Double, default: 10.0)

  This parameter indicates publish rate [Hz] of the constant frame.

* `~height` (Double, default: 1.0)

  This parameter indicates initial height [m] of the constant frame.

### Subscribing Topics

* `~height` (`std_msgs/Float64`)

  This topic modifies height [m] of the constant frame.

## util/initialpose_publisher.l

This script sets initial pose with relative pose from specified TF frame by publishing `/initialpose`.

### Parameters

* `~transform_base` (String, default: "map")

  TF frame of publishing topic `/initialpose`.
  
* `~transform_frame` (String, default: "eng2/7f/73B2")

  Base TF frame to calcurate relative initial pose
  
* `~initial_pose_x` (Double, default: 0.0)

  Relative pose x

* `~initial_pose_y` (Double, default: 0.0)

  Relative pose y

* `~initial_pose_yaw` (Double, default: 0.0)

  Relative pose yaw


### Subscribing Topics

* `/amcl_pose` (`geometry_msgs/PoseWithcovariancestamped`)
