# jsk\_baxter\_startup

The `jsk_baxter_startup` package.

## ROS launch and euslisp

### Default gripper

Launch and euslisp program for Baxter + default gripper

```bash
roslaunch jsk_baxter_startup baxter_default.launch
```

```bash
roscd baxtereus
roseus baxter-interface.l
;; euslisp interpreter
(baxter-init)
```

### Softhand gripper

Launch and euslisp program for Baxter + [Softhand](https://ieeexplore.ieee.org/document/8968011)

```bash
roslaunch jsk_baxter_startup baxter_softhand.launch
```

```bash
roscd baxtereus
roseus baxter-softhand-interface.l
;; euslisp interpreter
(baxter-init)
```

## ROS nodes

### xdisplay\_image\_topic.py

**Usage**

```bash
rosrun jsk_baxter_startup xdisplay_image_topic.py /cameras/head_camera/image
```

<img src="images/xdisplay_image_topic.jpg" width="30%" />
