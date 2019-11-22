# aero-ros-pkg-private

## Installation
Install aero-ros-pkg on ROS workspace
```
git clone https://github.com/seed-solutions/aero-ros-pkg
git clone https://github.com/taichiH/jsk_robot
```

## Install Dependencies on ROS workspace
```
rosdep update
rosdep install -y -r --from-paths src --ignore-src
```

## Build and setup aero_startup
```
catkin build aero_description
catkin build jsk_aero_shop
roscd aero_desctiption
./setup jsk_aero_shop/typeJSK
source ~/.bashrc
```

## Bringup Aero
Connect PC to JSK_AERO network
```
ssh aero_c
```
Launch files
```
roslaunch jsk_aero_private aero.launch
```

## View Robot Model
```
roslaunch jsk_aero_startup aero_rviz.launch
```

## Control from eus interface
Follow https://github.com/taichiH/jsk_robot/tree/master/jsk_aero_robot/aeroeus \
If you are JSK lab member, choose `JSK Robot Model` in `Create eusmodel`

```
roscd aeroeus
catkin build aeroeus
roseus aero-interface.l
(aero-init)
(setq *aero* *robot*)
(objects (list *robot*))
```

## Control SeedHand by angle
Before start angle control, you may need to send GraspAngle once.
```
rosservice call /aero_hand_controller "{hand: $(python -c 'import aero_startup.srv; print aero_startup.srv.HandControlRequest.HAND_RIGHT'), command: $(python -c 'import aero_startup.srv; print aero_startup.srv.HandControlRequest.COMMAND_GRASP'), power: 0, time_sec: 1.0}"
```
or
```
(send *ri* :start-grasp :rarm)
```

Set index finger
```
rosservice call /seedhand_server/set_hand "{finger: 'index', arm: 'rarm', thumb_angle: 0.0, index_angle: 0.0, time: 3.0}"
```
Set thumb finger
```
rosservice call /seedhand_server/set_hand "{finger: 'index', arm: 'rarm', thumb_angle: 0.0, index_angle: 0.0, time: 3.0}"
```

Set both finger
```
rosservice call /seedhand_server/set_hand "{finger: 'both', arm: 'rarm', thumb_angle: 0.05, index_angle: 0.3, time: 3.0}"
```

## Control from Joy
Control mode is divided into `ik-mode` and `basic-mode`.
Furthermore, `basic-mode` is divided into `joint-mode` and `base-mode`

In the standard, control mode is a `ik-mode`.

```
Push 1 (L-stick button): Change to a `basic-mode`

In basic-mode
Push 5 (options button): Switch to a `base-mode`
```

![ps4_controller](https://user-images.githubusercontent.com/22497720/54115852-ca9e4600-4430-11e9-82fb-06bf0c3f04b2.JPG)

![ps4_controller2](https://user-images.githubusercontent.com/22497720/54115856-cbcf7300-4430-11e9-9af1-0ed643d24059.JPG)

## IK mode
Control larm with 15 (L2 button) \\
Control rarm with 17 (R2 button) \\
Basically, solve fullbody inverse kinematics,
when you want to  keep another arm end-coords, control arm with 14 (L1 button) \\
Examples \\
Move larm forward
```
Forward 1 (L-stick) with 14 (L2 button)
```

Move larm Right
```
Rightward 2 (R-stick) with 14 (L2 button)
```

Rotate yaw
```
Push 13 or 11 with 14 (L2 button)
```

|Button|Function                        |
|:-----|:-------------------------------|
|0     |---                             |
|1     |Control Larm                    |
|2     |Control Rarm                    |
|3     |---                             |
|4     |---                             |
|5     |---                             |
|6     |Rotate Pitch -                  |
|7     |Rotate Role +                   |
|8     |Rotate Pitch +                  |
|9     |Rotate Role -                   |
|10    |Grasp                           |
|11    |Rotate Yaw +                    |
|12    |Open                            |
|13    |Rotate Yaw -                    |
|14    |---                             |
|15    |Larm prefix                     |
|16    |Keep arm prefix                 |
|17    |Rarm prefix                     |
