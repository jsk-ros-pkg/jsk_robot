# JSK Panda Robot
## Installation
- Following pages provide informative resources.
- Official manual page: https://frankaemika.github.io/docs/installation_linux.html
- Franka Community: https://www.franka-community.de/


### Installation for User PC
1. Install OpenHaptics from here: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US

2. Install ROS packages:
   ```bash
   mkdir -p ~/franka_ws/src
   cd ~/franka_ws/src
   wstool init
   wstool merge https://raw.githubusercontent.com/ykawamura96/jsk_robot/add_panda_robot/jsk_panda_robot/jsk_panda.rosinstall
   wstool update
   cd ../
   source /opt/ros/melodic/setup.bash
   rosdep install -y -r --from-paths src --ignore-src
   catkin build jsk_panda_startup jsk_panda_teleop
   source devel/setup.bash
   ```
### Installation for Panda Controller PC
1. Please see and follow installation written in: https://frankaemika.github.io/docs/installation_linux.html
   * Note that you need to install real-time kernel (`PREEMPT-PR` kernel) for real-time control.
   * Ref: Current controller PC uses following kernel: `5.4.19-rt11`
2. Please do the same ROS environment setup as `Installation for User PC` section above.



## Running Dual-Panda
### Boot robot
1. Please turn on the controller box and unlock joints by accessing desk.
### Via roseus
1. Start controller on controller PC:
   ```bash
   ssh leus@dual_panda.jsk.imi.i.u-tokyo.ac.jp
   roslaunch jsk_panda_startup dual_panda.launch
   ```

2. Controlling Dual-Panda via roseus:
   1. `rossetdualpanda`
   2. execute following script in roseus:
      ```lisp
      (load "package://panda_eus/euslisp/dual_panda-interface.l")
      (dual_panda-init)
      (send *robot* :angle-vector (send *robot* :reset-pose))
      (when (send *ri* :check-error)
        (send *ri* :recover-error))
      (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
      ```
      `(send *ri* :recover-error)` is required every time when you press and release the black switch (`activated` -> `monitored stop` -> `activated`).
### Teleop
#### Start program
1. Start controller on controller PC:
   ```bash
   ssh leus@dual_panda.jsk.imi.i.u-tokyo.ac.jp
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
   rossetdualpanda
   rosrun jsk_panda_teleop start_master_side.sh
   ```
   Then you should see rviz popup like this:
   ![image](https://user-images.githubusercontent.com/43567489/159150327-5e4d246b-2311-4eb4-814a-7f6fd11b6f29.png)

#### Move robot
1. Press connection mode switch (white button) to change connect or not-connect status between the haptic devices and the robot.
2. Press gripper switch to close / open gripper.

![image](https://user-images.githubusercontent.com/43567489/159150507-75122802-121e-4a22-abd1-b9540890950b.png)

#### Trouble Shooting
1. `Failed to initialize haptic device`  -> Please give access to haptic devices, i.e `sudo chmod 777 /dev/ttyACM[0-1]`

