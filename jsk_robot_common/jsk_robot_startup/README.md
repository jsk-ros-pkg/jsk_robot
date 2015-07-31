jsk_robot_startup
===

# lifelog/mongodb.launch

Launch file for logging data of robots.

## setup

### specify robot identifier

- set param for your own robot identifier:

```bash
rosparam set robot/name pr1012 # pr1040, baxter, pepper, etc...
```

- include `launch/mongodb.launch` in your robot startup launch file.
