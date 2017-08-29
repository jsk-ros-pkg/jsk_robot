jsk_robot_startup/lifelog
===

## mongodb.launch

Launch file for logging data of robots.

### setup

#### specify robot identifier

- set param for your own robot identifier:

```bash
rosparam set robot/name pr1012 # pr1040, baxter, pepper, etc...
```

- include `launch/mongodb.launch` in your robot startup launch file.

## action_logger.py

Save action goal, result and feedback to database

### Parameters

* `~update_cycle` (Double, default: `1.0`)

  spin rate
  
* `~white_list` (dict)

  White list of logging action. Topics are specified with `name` or `type`
  
  e.g.:
  
  ```yaml
name:
- /r_arm_controller/follow_joint_trajectory/result
type:
- JointTrajectoryActionGoal
- JointTrajectoryActionResult
- JointTrajectoryActionFeedback
- PointHeadActionGoal
- PointHeadActionResult
```

* `~black_list` (dict)

  Black list of logging action. Topics are specified with `name` or `type`

  see `~white_list` for example.

## base_trajectory_logger.py

Save base trajectory to database

### Parameters

* `~update_cycle` (Double, default: `1.0`)

  spin rate

* `~map_frame` (String, default: `map`)

  base static tf frame
  
* `~robot_frame` (String, default: `base_link`)

  robot base tf frame
  
## object_detection_logger.py

Save object detection result to database

### Parameters

* `~update_cycle` (Double, default: `1.0`)

  spin rate

* `~map_frame` (String, default: `map`)

  base static tf frame
  
* `~robot_frame` (String, default: `base_footprint`)

  robot base tf frame
