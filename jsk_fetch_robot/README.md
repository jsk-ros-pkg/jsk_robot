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

% You need to install `ros-ingio-jsk-tools` to use `rosset*` tools, otherwise use setenv command

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


fetch-interface function APIs
-----------------------------

- get current angle values of the robot model

  ```
(send *fetch* :angle-vector)
  ```

- get current angle values of tye real robot

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
(send *fetch* :start-grasp)
  ```
  to relace object

  ```
(send *fetch* :stop-grasp)
  ```


FAQ
---

- `could not fid package [fetcheus]`

  ```
1.irteusgl$ (load "package://fetcheus/fetch-interface.l") ; load modules
[rospack] Error: package 'fetcheus' not found
;; could not find pacakge [fetcheus]
/opt/ros/indigo/share/euslisp/jskeus/eus/Linux64/bin/irteusgl 0 error:  file "package://fetcheus/fetch-interface.l" not found in (error "file ~s not found" fname)
```

  You mgiht be forget to `source setup.bash` before you run `roseus`
