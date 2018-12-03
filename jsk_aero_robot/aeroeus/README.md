# aeroeus

This is EUSLISP interface to control aero.

## Build aeroeus

```
catkin build aeroeus  # nothing to build, to recognize from rospack
source ~/.bashrc
```

## Create eusmodel
```
rosrun euscollada collada2eus $(rospack find aero_description)/robots/aero.urdf $(rospack find aero_description)/robots/aero.yaml aero.l
```

## Run euslisp

```
roseus
$ load "package://aeroeus/aero-interface.l"
```

To create aero model
```
(aero-robot)
(objects (list *aero*))
```

To initialize eus interface,
```
(aero-init)
(objects (list *aero*))
```

Then, you can control AERO from euslisp, like

```
(send *aero* :reset-pose)
(send *ri* :angle-vector (send *aero* :angle-vector) 5000)
```

Solving iverse kinematics

```
(setq target-coords (send *aero* :larm :end-coords :copy-worldcoords))
(send *aero* :set-from-IK target-coords :arm {:larm :rarm :both-arm} :range {:arm :upper-body :whole-body} :end-effector {:hand :grasp :pick :index :thumb })
```

Both arm manipulatrion

```
(setq rtgt (send *aero* :rarm :end-coords :copy-worldcoords))
(setq ltgt (send *aero* :larm :end-coords :copy-worldcoords))
(send *aero* :inverse-kinematics (list rtgt ltgt) :ik-group :both-arm)
(send *aero* :set-from-IK (list rtgt ltgt) :arm :both-arm :range {:arm :upper-body :whole-body} :end-effector {:hand :grasp :pick :index :thumb })
```

Getting end-effector

```
(send *aero* :ik-target :arm {:larm :rarm :both-arm} :name {:hand :grasp :pick :index :thumb })
```
