jsk_spot_teleop
===============

This package is for teleoperation of Spot with a dualshock 3 controller.

## Prerequities

- Ubuntu 18.04 + ROS Melodic
- dualshock 3 controller

## How to run

Before using this pacakge, please connect a dualshock 3 controller with a computer

```bash
roslaunch jsk_spot_teleop teleop.launch
```

## Key Mapping

|Button   |Function                        |
|:--------|:-------------------------------|
|cross    | Stand                          |
|circle   | Sit                            |
|rectangle| Self Right                     |
|start    | Claim                          |
|select   | release                        |
|up       | power on                       |
|down     | power off                      |
|L1       | Unlock movement                |
|R2       | Unlock movement with fast mode |
|R1       |**EStop (gentle)**              |
