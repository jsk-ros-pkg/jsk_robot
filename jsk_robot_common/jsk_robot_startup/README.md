jsk_robot_startup
===

# lifelog/mongodb.launch

Launch file for logging data of robots.

## setup

### specify robot identifier

- open `~/.bashrc` and write your own robot identifier:

```bash
export ROBOT_NAME=pr1012 # pr1040, baxter, pepper, etc...
```

- include `launch/mongodb.launch` in your robot startup launch file.
