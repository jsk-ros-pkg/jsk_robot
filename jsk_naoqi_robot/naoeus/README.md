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
0:  :head :neck-y
1:  :head :neck-p
2:  :larm :shoulder-p
3:  :larm :shoulder-r
4:  :larm :elbow-y
5:  :larm :elbow-p
6:  :larm :wrist-y
7:  :rarm :shoulder-p
8:  :rarm :shoulder-r
9:  :rarm :elbow-y
10: :rarm :elbow-p
11: :rarm :wrist-y
12: :lleg :crotch-y
13: :lleg :crotch-r
14: :lleg :crotch-p
15: :lleg :knee-p
16: :lleg :ankle-p
17: :lleg :ankle-r
18: :rleg :crotch-y
19: :rleg :crotch-r
20: :rleg :crotch-p
21: :rleg :knee-p
22: :rleg :ankle-p
23: :rleg :ankle-r
```
