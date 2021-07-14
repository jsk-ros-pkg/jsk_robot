# jsk_fetch_robot

## Online Manual

[Fetch Robotics Online Manual](http://docs.fetchrobotics.com/FetchRobotics.pdf)

## Teleop Commands

|Button|Function                        |
|:-----|:-------------------------------|
|0     |Open gripper                    |
|1     |Control robot turning           |
|2     |Control forward/backward driving|
|3     |Close gripper                   |
|4     |Move arm linear                 |
|5     |Arm tuck                        |
|6     |Move arm angular                |
|7     |Disable motor position holding  |
|8     |Not used                        |
|9     |Unsafe teleop                   |
|10    |Primary deadman                 |
|11    |Head control deadman            |
|12    |Torso up                        |
|13    |Dock                            |
|14    |Torso down                      |
|15    |Undock                          |
|16    |Pair/unpair with robot          |
|9 & 11|**Software Runstop**            |

![joystick_numbered](https://user-images.githubusercontent.com/19769486/28101905-889e9cc2-6706-11e7-9981-5704cc29f2b3.png)
![joystick_numbered2](https://user-images.githubusercontent.com/19769486/28101906-88b5f20a-6706-11e7-987c-d94e64ac2cc1.png)


## How to Run


### Setup Environment

First, you need to install ros. For ros indigo, please refer to install guide like [here](http://wiki.ros.org/indigo/Installation/Ubuntu)

```bash
mkdir -p catkin_ws/src
cd  catkin_ws/src
wstool init .
wstool set --git jsk-ros-pkg/jsk_robot https://github.com/jsk-ros-pkg/jsk_robot.git -y
if [ $ROS_DISTRO = "indigo" ]; then
  wstool merge -t . https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_fetch_robot/jsk_fetch_user.rosinstall.indigo
elif [ $ROS_DISTRO = "kinetic" ]; then
  wstool merge -t . https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_fetch_robot/jsk_fetch_user.rosinstall.kinetic
fi
wstool update -t .
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build fetcheus jsk_fetch_startup
source devel/setup.bash
```

### Connecting to Fetch

You need to install `ros-indigo-jsk-tools` to use `rosset*` tools, otherwise use setenv command

```bash
rossetip             ## set ROS_IP and ROS_HOSTNAME
rossetmaster fetch15 ## set ROS_MASTER_URI
```

Inorder to confirm your shell environment, check with rviz

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch jsk_fetch_startup rviz.launch
```

### Control Fetch via roseus

Run `roseus` under emacs's shell environment (`M-x shell`).
When you start new shell, do not forget to run `rossetip`, `rossetmaster fetch15` and `source ~/catkin_ws/devel/setup.bash`

```lisp
(load "package://fetcheus/fetch-interface.l") ;; load modules
(setq *fetch* (fetch))          ;; creat a robot model
(objects (list *fetch*))        ;; display the robot model
(setq *ri* (instance fetch-interface :init)) ;; make connection to the real robot
```

or

```lisp
(load "package://fetcheus/fetch-interface.l") ;; load modules
(fetch-init)
```

### Gazebo Simulation

Gazebo can be used for simulating robot's movements.
It is unable to communicate to the real robot when running the simulation.

```bash
roslaunch fetch_gazebo simulation.launch
roslaunch fetch_moveit_config move_group.launch
```

## Fetcheus APIs

### Common

- Get current angle values of the robot model

```lisp
(send *fetch* :angle-vector)
```

- Get current angle values of the real robot

```lisp
(send *ri* :state :potentio-vector)
```

- Update joint angle values of the robot model from the real robot

```lisp
(send *fetch* :angle-vector (send *ri* :state :potentio-vector))
```

- Set angle values to the robot model

```lisp
(send *fetch* :angle-vector #f(150 75 80 -10 100 0 95 0 0 0))
```

- Set `reset-pose` to the robot model

```lisp
(send *fetch* :reset-pose)
```

- Send current joint angles of robot model to real robot in 5000 \[ms\] (CAUTION, this will move the real robot)

```lisp
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
```

- Send current head joint angles of robot model to real robot, this command only control joints of head limb. (DO NOT USE THIS COMMAND EXCEPT **HEAD**)

 ```lisp
(send *fetch :head :neck-y :joint-angle 50)
(send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
(send *ri* :wait-interpolation)
 ```

- Get angle value of individual joint

```lisp
(send *fetch* :torso :waist-z :joint-angle)
(send *fetch* :rarm :shoulder-y :joint-angle)
(send *fetch* :rarm :shoulder-p :joint-angle)
(send *fetch* :rarm :shoulder-r :joint-angle)
(send *fetch* :rarm :elbow-p :joint-angle)
(send *fetch* :rarm :elbow-r :joint-angle)
(send *fetch* :rarm :wrist-p :joint-angle)
(send *fetch* :rarm :wrist-r :joint-angle)
(send *fetch* :head :neck-y :joint-angle)
(send *fetch* :head :neck-p :joint-angle)
```

- Set angle value of indiviual joint, use \[mm\] for linear joint and \[deg\] for rotational joint

```lisp
(send *fetch* :torso :waist-z :joint-angle 150)
(send *fetch* :rarm :shoulder-y :joint-angle 75)
```

### Gripper Control

- Grasp object

```lisp
(send *ri* :start-grasp)
```

- Release object

```lisp
(send *ri* :stop-grasp)
```

- Know the result of grasp

```lisp
(setq grasp-result (send *ri* :start-grasp))
(send grasp-result :position)
```

### Inverse Kinematics

- Including torso movement

```lispj
(send *fetch* :inverse-kinematics (make-coords :pos #f(800 0 1200)) :debug-view t)
```

- Using only arm
```lisp
(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(800 0 1200)) :degug-view t)
```

coordinates can be made with
```lisp
(make-coords :pos #f(0 0 0) :rpy (float-vector pi 0 0))
```

### Mobile Base

- Move 1 [m] forward, based on relative position (CAUTION, this will move the real robot)

```lisp
(send *ri* :go-pos 1 0 0)
```

- Move to position `#f(-1000 7000 0)` [mm], based on absolute map positions (CAUTION, this will move the real robot)

```lisp
(send *ri* :move-to (make-coords :pos #f(-1000 7000 0) :rpy (float-vector pi/2 0 0)) :frame-id "/map") ;; :retry 1 ;;[mm]
```

- Move forward at 0.1[m/s] for 10 seconds (CAUTION, this will move the real robot)

```lisp
(send *ri* :go-velocity 0.1 0 0 10000)
```

- Get current position of the real robot
```lisp
(send *ri* :state :worldcoords)
```
Please see [costmap_2d](http://wiki.ros.org/costmap_2d?distro=melodic), [move_base](http://wiki.ros.org/move_base?distro=melodic), if you would like to understand the self-positioning system in detail.

### Speak

- Use text-to-speech engine to speak text

```lisp
(send *ri* :speak "hello")
(send *ri* :speak (format nil "hello, ~A + ~A is ~A" 1 1 (+ 1 1)))
(send *ri* :speak-jp "こんにちは")
```

## FAQ

- `could not find package [fetcheus]`

```
1.irteusgl$ (load "package://fetcheus/fetch-interface.l") ; load modules
[rospack] Error: package 'fetcheus' not found
;; could not find pacakge [fetcheus]
/opt/ros/indigo/share/euslisp/jskeus/eus/Linux64/bin/irteusgl 0 error:  file "package://fetcheus/fetch-interface.l" not found in (error "file ~s not found" fname)
```

You might be forget to `source setup.bash` before you run `roseus`

- Can't connect joystick to Fetch.

Sometimes, we encounter the problem of connecting joystick with Fetch.
There is an instruction in [the official documentation](http://docs.fetchrobotics.com/faq.html#why-won-t-my-robot-move-when-i-use-my-ps3-joystick),
but we found following is also effective if the official one does not work.  
Following [ps3joy documentation](http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle),

```bash
ssh fetch15
sudo bash
rosrun ps3joy sixpair
rosrun ps3joy ps3joy.py  # with pushing the center button of the joystick
```

  You might be forget to `source setup.bash` before you run `roseus`

Administration
--------------

- set global environment variables https://github.com/jsk-ros-pkg/jsk_robot/issues/859#issuecomment-341269420
```
$ cat /etc/profile.d/jsk.sh 
# added by furushchev (2017.11.2)
export ROSLAUNCH_SSH_UNKNOWN=1                 # enable to run roslaunch with " Server' not found in known_hosts" 
export JSK_DATA_CACHE_DIR=/etc/ros/jsk_data    # store recognition data within common directories to reduce hdd usage
```

**NOTE**  
Scripts located in `/etc/profile.d` will be enabled on next login to the shell, so it is necessary to first log out and re-login to apply this change to current users.

- Create directory for cache data for JSK repository

```bash
$ sudo mkdir /etc/ros/jsk_data && sudo chmod 0777 /etc/ros/jsk_data
```

- change permissoin of log direcotry https://github.com/jsk-ros-pkg/jsk_robot/issues/859#issuecomment-341269420

`logrotate` does not work correctly due to directory permission
```
$ sudo /usr/sbin/logrotate /etc/logrotate.d/ros
error: skipping "/var/log/ros/fd645e8c-9a09-11e5-8547-d8cb8a40210c/head_camera-depth_registered_rectify_depth-12-stdout.log" because parent directory has insecure permissions (It's world writable or writable by group which is not "root") Set "su" directive in config file to tell logrotate which user/group should be used for rotation.
```
Changed `/var/log/ros` manually
```
sudo chmod g-w /var/log/ros
(cd /var/log/ros && find -type d | xargs sudo chmod g-w)
```
c.f.
```
furushchev@fetch15:/var/log/ros$ ls -lFahd
drwxrwsr-x 381 ros ros 36K Nov  2 01:45 ./
furushchev@pr2:/var/log/ros$ ls -lFahd
drwxr-xr-x 6 ros ros 36K 11月  1 15:25 ./
```

- change script for auto `undocking` to disable auto rotatation after unplugged

```diff
# /opt/ros/indigo/lib/fetch_auto_dock/undock_on_button.py
44c44,47
<         goal.rotate_in_place = True
---
>         # fixed by furushchev (2017/11/1)
>         # Disabled rotate in place feature
>         # goal.rotate_in_place = True
>         goal.rotate_in_place = False
```

- Set branch of `jsk_demos` as `master` in order to update jsk_maps

- Add `pr2eus` package to `/home/fetch/ros/indigo/src/jsk-ros-pkg` to use [the newest `:speak` function](https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/332). This function will be released in `0.3.14` of `jsk_pr2eus` in the future.

- Set `amcl/odom_alpha*` paramters manually in `/opt/ros/indigo/share/fetch_navigation/launch/include/amcl.launch`, to prevent jumps of self-position of fetch. Please see [this Pull Request](https://github.com/fetchrobotics/fetch_ros/pull/76). This is released in `0.7.14` of `fetch_ros`.

- Fix wifi access point to catch access point with strong radio wave intensity

- To add `cartesian_wrench_controller` to `default controller`,

```
source /home/fetch/fetch_controller_ws/devel/setup.bash # in /home/fetch/ros/indigo_robot/devel/setup.bash
```

- To use `respeaker` and `julius_ros` (`2.1.10` of `jsk_3rdparty`)

```
source /home/fetch/audio_ws/devel/setup.bash # in /home/fetch/ros/indigo/devel/setup.bash
```
