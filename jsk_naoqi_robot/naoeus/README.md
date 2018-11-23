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
2. Please refer to `Control NAO via roseus`.
3. Please try methods, you can refer to the explanations below how to try them.

Methods
-------

TODO
