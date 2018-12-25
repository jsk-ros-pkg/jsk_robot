naoeus
======

This is a NAO-specific package for roseus interface.

How to make nao model on euslisp
--------------------------------

Install nao mesh files from deb with manual approval of license
```
sudo apt-get install ros-<ros version>-nao-meshes 
catkin build
```

Control NAO via roseus
-------------------------

```
(load "package://naoeus/nao-interface.l") ;; load modules
(setq *nao* (nao))          ;; creat a robot model
(setq *ri* (instance nao-interface :init)) ;; make connection to the real robot
(objects (list *nao*))        ;; display the robot model
```
or

```
(load "package://naoeus/nao-interface.l") ;; load modules
(nao-init)
```

How to try methods
------------------

1. [roslaunch jsk_nao_startup.launch](../jsk_nao_statup/README.md)  
2. Please refer to [Control NAO via roseus](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/naoeus/README.md#control-nao-via-roseus).  
3. Please try methods, you can refer to the explanations below how to try them. If there is a sign of `kochigami-develop`, please follow (Interface when controlling NAO and Pepper via roseus)[../README.md#interface-when-controlling-nao-and-pepper-via-roseus]. 

Joints of NAO
-------------

Here is a list of joints when accessing NAO. ex:

```
(send *nao* :reset-pose)
=> #f(110.0 10.0 -90.0 -40.0 0.0 110.0 -10.0 90.0 40.0 0.0 0.0 0.0 -30.0 60.0 -30.0 0.0 0.0 0.0 -30.0 60.0 -30.0 0.0 0.0 0.0)
       0     1     2     3    4   5      6    7    8    9  10   11   12   13     14  15  16  17   18   19    20   21  22  23
```

```
0:  :larm :shoulder-p
1:  :larm :shoulder-r
2:  :larm :elbow-y
3:  :larm :elbow-p
4:  :larm :wrist-y
5:  :rarm :shoulder-p
6:  :rarm :shoulder-r
7:  :rarm :elbow-y
8:  :rarm :elbow-p
9:  :rarm :wrist-y
10: :lleg :crotch-y
11: :lleg :crotch-r
12: :lleg :crotch-p
13: :lleg :knee-p
14: :lleg :ankle-p
15: :lleg :ankle-r
16: :rleg :crotch-y
17: :rleg :crotch-r
18: :rleg :crotch-p
19: :rleg :knee-p
20: :rleg :ankle-p
21: :rleg :ankle-r
22: :head :neck-y
23: :head :neck-p
```