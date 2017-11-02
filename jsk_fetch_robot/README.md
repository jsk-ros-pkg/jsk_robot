jsk_fetch_robot
================

setup environment
-----------------
% First, you need to install ros. For ros indigo, please refer to install guide like [here](http://wiki.ros.org/indigo/Installation/Ubuntu)

```
mkdir -p catkin_ws/src
cd  catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_fetch_robot/jsk_fetch.rosinstall
wstool update -t src
source /opt/ros/indigo/setup.bash
rosdep install -y -r --from-paths src --ignore-src
catkin build fetcheus jsk_fetch_startup
source devel/setup.bash
```

connecting to the robot
-----------------------

% You need to install `ros-indigo-jsk-tools` to use `rosset*` tools, otherwise use setenv command

```
rossetip             % set ROS_IP and ROS_HOSTNAME
rossetmaster fetch15 % set ROS_MASTER_URI
```

to confirm your shell environment, check with rviz
```
source ~/catkin_ws/devel/setup.bash
roslaunch jsk_fetch_robot rviz.launch
```

control robot via roseus
------------------------

run `roseus` under emacs's shell environment (`M-x shell`), when you start new shell, do not forget to run `rossetip`, `rossetmaster fetch15` and `source ~/catkin_ws/devel/setup.bash`

```
(load "package://fetcheus/fetch-interface.l") ;; load modules
(setq *fetch* (fetch))          ;; creat a robot model
(objects (list *fetch*))        ;; display the robot model
(setq *ri* (instance fetch-interface :init)) ;; make connection to the real robot
```

or run `(fetch-init)` after you load `"package://fetcheus/fetch-interface.l"`

gazebo simulation
------------------------

Can be used for simulating robot's movements. Unable to communicate to the real robot when running the simulation.

first, in one terminal
```
roslaunch fetch_gazebo simulation.launch
```
then, in another terminal
```
roslaunch fetch_moveit_config move_group.launch
```

fetch-interface function APIs
-----------------------------

- get current angle values of the robot model

```
(send *fetch* :angle-vector)
```

- get current angle values of the real robot

```
(send *ri* :state :potentio-vector)
```

- update joint angle values of the robot model from the real robot

```
(send *fetch* :angle-vector (send *ri* :state :potentio-vector))
```

- set angle values to the robot model

```
(send *fetch* :angle-vector #f(150 75 80 -10 100 0 95 0 0 0))
```

- set "reset pose" to the robot model

```
(send *fetch* :reset-pose)
```

- send current joint angles of robot model to real robot in 5000 \[ms\] (CAUTION, this will move the real robot)

```
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
```

- send current head joint angles of robot model to real robot, this command only control joints of head limb. (DO NOT USE THIS COMMAND EXCEPT **HEAD**)

 ```
(send *fetch :head :neck-y :joint-angle 50)
(send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
(send *ri* :wait-interpolation)
 ```

- get angle value of individual joint
```
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

- set angle value of indiviual joint, use \[mm\] for linear joint and \[deg\] for rotational joint

```
(send *fetch* :torso :waist-z :joint-angle 150)
(send *fetch* :rarm :shoulder-y :joint-angle 75)
```

- control gripper

  to grasp object
```
(send *ri* :start-grasp)
```
  
  to release object
```
(send *ri* :stop-grasp)
```

  to know the result of grasp
```
(setq grasp-result (send *ri* :start-grasp))
(send grasp-result :position)
```
  
- inverse kinematics

  including torso movement
```
(send *fetch* :inverse-kinematics (make-coords :pos #f(800 0 1200)) :debug-view t)
```
  
  using only arm movement
```
(send *fetch* :rarm :inverse-kinematics (make-coords :pos #f(800 0 1200)) :degug-view t)
```
  
  coordinates can be made with 
```
  (make-coords :pos #f(0 0 0) :rpy (float-vector pi 0 0))
```
  
- moving the robot
 
 move 1 [m] forward, based on relative position (CAUTION, this will move the real robot)
 ```
 (send *ri* :go-pos 1 0 0)
 ```
 
 move to point (-1000 7000 0) [mm], based on absolute map positions (CAUTION, this will move the real robot)
 ```
 (send *ri* :move-to (make-coords :pos #f(-1000 7000 0) :rpy (float-vector pi/2 0 0)) :frame-id "/map") ;; :retry 1 ;;[mm]
 ```
 
- use text-to-speech engine to speak text
 
 ```
  (send *ri* :speak "hello")
  (send *ri* :speak (format nil "hello, ~A + ~A is ~A" 1 1 (+ 1 1)))
 ```

FAQ
---

- `could not find package [fetcheus]`

```
1.irteusgl$ (load "package://fetcheus/fetch-interface.l") ; load modules
[rospack] Error: package 'fetcheus' not found
;; could not find pacakge [fetcheus]
/opt/ros/indigo/share/euslisp/jskeus/eus/Linux64/bin/irteusgl 0 error:  file "package://fetcheus/fetch-interface.l" not found in (error "file ~s not found" fname)
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
