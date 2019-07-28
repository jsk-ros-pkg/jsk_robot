# jsk\_baxter\_startup

The `jsk_baxter_startup` package.

## ROS launch

```bash
roslaunch jsk_baxter_startup baxter.launch load_robot_description:=true launch_robot_state_publisher:=true
```


## ROS nodes

### xdisplay\_image\_topic.py

**Usage**

```bash
rosrun jsk_baxter_startup xdisplay_image_topic.py /cameras/head_camera/image
```

<img src="images/xdisplay_image_topic.jpg" width="30%" />
