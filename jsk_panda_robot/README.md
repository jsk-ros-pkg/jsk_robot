# JSK Panda Robot
## Installation
- Following pages provide informative resources.
- Official manual page: https://frankaemika.github.io/docs/installation_linux.html
- Franka Community: https://www.franka-community.de/


### Installation for User PC
1. Install OpenHaptics and Touch Device Driver from here: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US
   * If your Ubuntu version is not supported, check JSK backup: https://drive.google.com/drive/folders/1FiQ4m3XtoDlwdRIq3H7LJv8T882SBVBl

2. Install ROS packages:
   ```bash
   mkdir -p ~/franka_ws/src
   cd ~/franka_ws/src
   wstool init
   wstool merge https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_panda_robot/jsk_panda_user.rosinstall
   wstool update
   cd ../
   source /opt/ros/melodic/setup.bash
   rosdep install -y -r --from-paths src --ignore-src --skip-keys=librealsense2,realsense2_camera  # See comments in jsk_panda.rosinstall
   catkin build jsk_panda_startup jsk_panda_teleop
   source devel/setup.bash
   ```
3. If you want to use TouchUSBs (haptic devices), see [this README](https://github.com/pazeshun/Geomagic_Touch_ROS_Drivers/tree/dual-phantom-readme#use-multiple-devices) to setup devices.
   * Note that this README was written for old Ubuntu.
     - On Ubuntu 18.04, run `Touch_Setup` instead of `/opt/geomagic_touch_device_driver/Geomagic_Touch_Setup`. If you want to check device status, run `Touch_Diagnostic` instead of `/opt/geomagic_touch_device_driver/Geomagic_Touch_Diagnostic`.

### Installation for Panda Controller PC
1. Please see and follow installation written in: https://frankaemika.github.io/docs/installation_linux.html
   * Note that you need to install real-time kernel (`PREEMPT-PR` kernel) for real-time control.
   * Ref: Current controller PC uses following kernel:  `dual-panda1`: `5.4.19-rt11`, `dual-panda2`: `5.4.93-rt51`
2. Install librealsense2:
   https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
3. Install OpenHaptics and Touch Device Driver from here: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US
   * If your Ubuntu version is not supported, check JSK backup: https://drive.google.com/drive/folders/1FiQ4m3XtoDlwdRIq3H7LJv8T882SBVBl
4. Install ROS packages:
   ```bash
   mkdir -p ~/franka_ws/src
   cd ~/franka_ws/src
   wstool init
   wstool merge https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_panda_robot/jsk_panda.rosinstall
   wstool update
   cd ../
   source /opt/ros/melodic/setup.bash
   rosdep install -y -r --from-paths src --ignore-src --skip-keys=librealsense2,realsense2_camera  # See comments in jsk_panda.rosinstall
   catkin build jsk_panda_startup jsk_panda_teleop
   source devel/setup.bash
   ```


## Running Dual-Panda
### Boot robot
1. Please turn on the controller box and unlock joints by accessing desk.
### Via roseus
1. Start controller on controller PC:
   ```bash
   # dual_panda1: robot having Xtion camera on head, web cameras on both hands, microphone on right hand
   # dual_panda2: robot having RealSense D435 on both hands
   ssh leus@dual-panda1.jsk.imi.i.u-tokyo.ac.jp  # Or ssh leus@dual-panda2.jsk.imi.i.u-tokyo.ac.jp
   roslaunch jsk_panda_startup dual_panda1.launch  # Or roslaunch jsk_panda_startup dual_panda2.launch
   ```

2. Controlling Dual-Panda via roseus:
   1. Setting up network:
      ```bash
      rossetmaster dual-panda1.jsk.imi.i.u-tokyo.ac.jp  # Or rossetmaster dual-panda2.jsk.imi.i.u-tokyo.ac.jp
      rossetip
      ```
   2. Execute following script in roseus:
      ```lisp
      (load "package://panda_eus/euslisp/dual_panda-interface.l")
      (dual_panda-init)
      (send *robot* :angle-vector (send *robot* :reset-pose))
      (when (send *ri* :check-error)
        (send *ri* :recover-error))
      (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
      ```
      `(send *ri* :recover-error)` is required every time when you press and release the black switch (`activated` -> `monitored stop` -> `activated`).
#### Record/play rosbag
```bash
roslaunch jsk_panda_startup dual_panda1_record.launch  # Or roslaunch jsk_panda_startup dual_panda2_record.launch
# Generated bag file: /tmp/panda_rosbag/data_*.bag
roslaunch jsk_panda_startup dual_panda1_play.launch bagfile_name:=/path/to/bagfile  # Or roslaunch jsk_panda_startup dual_panda2_play.launch bagfile_name:=/path/to/bagfile
```
### Teleop
#### Start program
1. Start controller on controller PC:
   ```bash
   ssh leus@dual-panda1.jsk.imi.i.u-tokyo.ac.jp
   roslaunch jsk_panda_teleop start_panda_teleop_follower_side.launch start_bilateral:=true
   ```
   `start_bilateral:=true` connects haptic device and dual_panda from the beginning, i.e. it moves the robot immediately after running leader (user) side.
   If you start with `start_bilateral:=false`, you would need to call following service call:
   ```bash
   rosservice call /dual_panda/control_bilateral "pose_connecting: true
     force_connecting: true
     reset_phantom: true
     wait: 3.0"
   ```

2. Start user PC:
   ```bash
   rossetmaster dual-panda1.jsk.imi.i.u-tokyo.ac.jp
   rossetip
   rosrun jsk_panda_teleop start_master_side.sh
   ```
   Then you should see rviz popup like this:
   ![image](https://user-images.githubusercontent.com/14994939/181154686-cb8e95c8-96a7-47ac-b074-bf3c72bc5ee8.png)

#### Move robot
1. Press connection mode switch (white button) to change connect or not-connect status between the haptic devices and the robot.
2. Press gripper switch to close / open gripper.

![image](https://user-images.githubusercontent.com/43567489/159150507-75122802-121e-4a22-abd1-b9540890950b.png)

#### Record/play rosbag
```bash
roslaunch jsk_panda_teleop panda_teleop_record.launch
# Generated bag file: /tmp/panda_rosbag/data_*.bag
roslaunch jsk_panda_teleop panda_teleop_play.launch bagfile_name:=/path/to/bagfile
```

#### Trouble Shooting
1. `Failed to initialize haptic device`  -> Please give access to haptic devices, i.e `sudo chmod 777 /dev/ttyACM[0-1]`

