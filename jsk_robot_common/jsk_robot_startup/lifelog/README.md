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


## Examples

You can start a mongodb logging system using `lifelog/mongodb.launch` and `lifelog/common_logger.launch` files.
 See [jsk_pr2_lifelog/db_client.launch](https://github.com/jsk-ros-pkg/jsk_robot/blob/5d90d483aaa674d33968f34db83f53cd3d018bd4/jsk_pr2_robot/jsk_pr2_startup/jsk_pr2_lifelog/db_client.launch), [fetch_lifelog.xml](https://github.com/jsk-ros-pkg/jsk_robot/blob/921097b7a9c16cd99d0eb8f9f271bda4784dadc5/jsk_fetch_robot/jsk_fetch_startup/launch/fetch_lifelog.xml) and [jsk_baxter_lifelog/db_client.launch](https://github.com/jsk-ros-pkg/jsk_robot/blob/c03dc5af06d8b7786b4212b132047acaa229eb0e/jsk_baxter_robot/jsk_baxter_startup/jsk_baxter_lifelog/db_client.launch) for examples.

## Sample client

Sample program to retrieve stored data can be found at [jsk_robot_startup/scripts/robot_databse_monogo_example.py](./scripts/robot_databse_monogo_example.py).

To connect replicator server (musca), you need to start [jsk_robot_startup/launch/robot_databse_monogo_server.launch](./launch/robot_databse_monogo_server.launch). Please be careful about `ROS_MASTER_URI`, because `robot_databse_monogo_server.launch` will starts same node name as robot system nodes, thus it may corrupt some settings.

## Troubleshooting

If you encounter following error
```
[ERROR] [1666693339.502586]: Could not get message store services. Maybe the message store has not been started? Retrying..
[ERROR] [1666693339.502586]: Could not get message store services. Maybe the message store has not been started? Retrying..
```

1) Is mongodb service working correctly?
```
sudo systemctl status mongodb.service
sudo systemctl start mongodb.service
```

2) Is `mongo` command works?
```
mongo localhost:27017
```

3) Check /etc/mongodb.conf
```
$ cat /etc/mongodb.conf | grep -v ^#
dbpath=/var/lib/mongodb
logpath=/var/log/mongodb/mongodb.log
logappend=true
bind_ip = 0.0.0.0
journal=true
```
Default `bind_jp` is `127.0.0.1`, and it not work on some machines, see [#1706](https://github.com/jsk-ros-pkg/jsk_robot/issues/1706) for more info.
