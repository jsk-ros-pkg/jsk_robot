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

## action_result_db.py

Save action result to mongodb

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
- PointHeadActionGoal
- PointHeadActionResult
```

* `~black_list` (dict)

  Black list of logging action. Topics are specified with `name` or `type`

  see `~white_list` for example.

## move_base_db.py

Save base trajectory to mongodb

### Parameters

* `~update_cycle` (Double, default: `1.0`)

  spin rate

* `~map_frame` (String, default: `map`)

  base static tf frame
  
* `~robot_frame` (String, default: `base_link`)

  robot base tf frame
  
## objectdetection_db.py

Save object detection result to mongodb

### Parameters

* `~update_cycle` (Double, default: `1.0`)

  spin rate

* `~map_frame` (String, default: `map`)

  base static tf frame
  
* `~robot_frame` (String, default: `base_footprint`)

  robot base tf frame
