peppereus
=========

How to make pepper model on euslisp
-----------------------------------

Install pepper mesh files with manual approval of license
```
sudo apt-get install ros-<ros version>-pepper-meshes
catkin build
```

Control Pepper via roseus
-------------------------

```
(load "package://peppereus/pepper-interface.l") ;; load modules
(setq *pepper* (pepper))          ;; creat a robot model
(setq *ri* (instance pepper-interface :init)) ;; make connection to the real robot
(objects (list *pepper*))        ;; display the robot model
```
or

```
(load "package://peppereus/pepper-interface.l") ;; load modules
(pepper-init)
```

Joints of Pepper
----------------

Here is a list of joints when accessing Pepper.
ex:
```
(send *pepper* :reset-pose)
=> #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)
       0    1    2   3    4     5     6     7    8     9    10   11   12   13  14
```
```
0: :knee-p
1: :hip-r
2: :hip-p
3: :larm :shoulder-p
4: :larm :shoulder-r
5: :larm :elbow-y
6: :larm :elbow-p
7: :larm :wrist-y
8: :rarm :shoulder-p
9: :rarm :shoulder-r
10: :rarm :elbow-y
11: :rarm :elbow-p
12: :rarm :wrist-y
13: :head :neck-y
14: :head :neck-p
```