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

![SpotDocumentation](https://user-images.githubusercontent.com/9410362/111890520-68b84400-8a2d-11eb-8f54-dcc6ac7ccbbb.png)

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
