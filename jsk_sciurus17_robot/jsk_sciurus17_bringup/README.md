# jsk\_sciurus17\_bringup

The `jsk_sciurus17_bringup` package.

## System Requirements

This package has been tested on the following environments:

- ROS Melodic
  - OS: Ubuntu 18.04 LTS
- ROS Noetic
  - OS: Ubuntu 20.04 LTS

## Installation

### Required Packages

This package depends on the following packages:

- [sciurus17_ros](https://github.com/rt-net/sciurus17_ros)
- [sciurus17_description](https://github.com/rt-net/sciurus17_description) (RT Corporation's non-commercial license applies)
- sciurus17eus (for EusLisp interface)

### Installation from Source

Install the required dependencies:

```bash
cd ~/catkin_ws/src

# Clone sciurus17_ros
git clone https://github.com/rt-net/sciurus17_ros.git

# Clone sciurus17_description
git clone https://github.com/rt-net/sciurus17_description.git

# Install dependencies
rosdep install -r -y --from-paths . --ignore-src

# Build
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Communication Device Setup

Configure the USB serial device name for `sciurus17_control` to communicate with the real robot:

```bash
roscd sciurus17_tools/scripts/
./create_udev_rules
```

After executing the script, reboot the system and connect Sciurus17. The device `/dev/sciurus17spine` will be created.

### RealSense Camera Setup

For stable operation of the head camera (RealSense D415), reset the camera before use:

```bash
rosrun sciurus17_tools realsense_hwreset
```

## ROS launch and euslisp

### Default launch

Launch for Sciurus17 with MoveIt, head camera, and chest camera

```bash
roslaunch jsk_sciurus17_bringup sciurus17_bringup.launch
```

#### Launch options

- `use_rviz`: Launch with RViz visualization (default: true)
- `use_head_camera`: Enable RealSense head camera (default: true)
- `use_chest_camera`: Enable chest camera (default: true)

Example with custom options:

```bash
roslaunch jsk_sciurus17_bringup sciurus17_bringup.launch use_rviz:=false use_head_camera:=false
```

### EusLisp interface

Launch and euslisp program for Sciurus17

```bash
roslaunch jsk_sciurus17_bringup sciurus17_bringup.launch
```

```bash
roscd sciurus17eus
roseus sciurus17-interface.l
;; euslisp interpreter
(sciurus17-init)
```

## Camera configuration

### Head camera (RealSense)

The head camera uses RealSense D415.
Camera topics are published under `/sciurus17/` namespace.

### Chest camera

The chest camera is launched separately and provides additional viewing angles for manipulation tasks.
