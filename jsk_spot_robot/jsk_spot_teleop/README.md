jsk_spot_teleop
===============

This package is for teleoperation of Spot with a dualshock 3 controller.

## Prerequities

- Ubuntu 18.04 + ROS Melodic
- dualshock 3 controller or Magicsee R1 controller

and please install udev rules to the computer.

```bash
roscd jsk_spot_teleop
sudo cp ./config/udev/* /etc/udev/rules.d/
```

## How to run

Before using this pacakge, please connect a controller to a computer

```bash
roslaunch jsk_spot_teleop teleop.launch pad_type:=<your pad type (dualshock3 or R1)>
```

## Key Mapping

### Dualshock 3

![SpotTeleopDualshock3](https://user-images.githubusercontent.com/9410362/114705406-ac6b4880-9d62-11eb-874b-bcec85f5c9f7.png)

|Button   |Function                        |
|:--------|:-------------------------------|
|cross    | Stand                          |
|circle   | Sit                            |
|triangle | Switch locomomotion mode       |
|rectangle| Self Right                     |
|start    | Claim                          |
|select   | release                        |
|up       | power on                       |
|down     | power off                      |
|right    | (Not assigned)                 |
|left     | toggle stair mode              |
|L1       | Unlock movement                |
|L2       | (Not assigned)                 |
|R1       | **EStop (gentle)**             |
|R2       | Unlock movement with fast mode |

### Magicsee R1

![SpotTeleopR1](https://user-images.githubusercontent.com/9410362/114705443-b55c1a00-9d62-11eb-82a5-a32a1ca61eb0.png)

|Button   |Function                        |
|:--------|:-------------------------------|
| Enter   | Unlock movement                |
| Back    | **EStop (gentle)**             |
| A       | Sit                            |
| B       | Stand                          |
| C       | claim                          |
| D       | power on                       |
