# jsk\_baxter\_startup

The `jsk_baxter_startup` package.

## Installation

### Copy `env.sh` to home dir for `baxter-c1`

Copy `env.sh` to home directory for machine tag in baxter-c1

```bash
ssh baxter-c1
roscd jsk_baxter_startup/jsk_baxter_machine
cp env.sh ~/
```

### Copy udev to `/etc/udev/rules.d`

```bash
roscd jsk_baxter_startup/jsk_baxter_udev
sudo cp * /etc/udev/rules.d
```

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

### Rosbag record

```bash
roslaunch jsk_baxter_startup baxter_rosbag_record.launch
```

### Rosbag play

```bash
roslaunch jsk_baxter_startup baxter_rosbag_play.launch
```

## ROS nodes

### xdisplay\_image\_topic.py

**Usage**

```bash
rosrun jsk_baxter_startup xdisplay_image_topic.py /cameras/head_camera/image
```

<img src="images/xdisplay_image_topic.jpg" width="30%" />
