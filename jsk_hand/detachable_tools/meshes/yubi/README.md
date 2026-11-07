# YUBI XC330

Imported from [jsk_yubi_description on Google Drive](https://drive.google.com/drive/folders/18jtXPjzfbRky48TK0_evYM04YtHQ9NiR), within the supplied [jsk_yubi folder](https://drive.google.com/drive/folders/1jPiEbtqSpN0F90bj-gPIDMsSRcsfGLF-), on 2026-11-07.

The original `yubi_xc330_front_flat_with_interface.stl` is included unchanged.
URDF mesh scale is 0.001 (CAD millimeters to ROS meters). Both hands use the
same supplied geometry; the source does not contain separate mirrored models.
The mesh includes the detachable interface and is a single static assembly.
The control-only joint retains the source axis, origin, and limits (-1.2 to
1.2 rad); moving it does not animate the fingers. The fixed mount uses the
source base frame with identity transform; its physical attachment transform
still needs verification on the robot.

## Motor IDs and control

| Side | Port | Motor ID |
| --- | --- | ---: |
| Left | `/dev/lhand` | 8 |
| Right | `/dev/rhand` | 9 |

IDs 0–7 remain assigned to the existing tools. These are configuration
assignments; importing the model does not rewrite the IDs in the motors.
Set the physical motors to IDs 8 and 9 before hardware launch.

The integration uses the package's existing position + effort interfaces,
XC330 current-based position mode 5, and default 57600 baud on both ROS versions.
This differs from the source's position-only mode 3 and 1000000 baud.
Match the bus baud rate to the motors (or override `baud_rate`). Hardware
operation and open/close angles have not been calibrated; no named open/close
positions are supplied for YUBI. Command angles explicitly within the limits.

## ROS 1

```bash
roslaunch detachable_tools dual_hand.launch left_tool:=yubi right_tool:=yubi
# Model only:
roslaunch detachable_tools hand_model.launch side:=lhand tool:=yubi
```

ROS 1 uses `lhand_joint` / `rhand_joint`, SimpleTransmission, and the existing
position trajectory and effort controllers. `dynamixel_info` is generated
from the IDs in `urdf/lhand_yubi.urdf` and `urdf/rhand_yubi.urdf`.

## ROS 2

```bash
ros2 launch detachable_tools dual_hand.launch.xml left_tool:=yubi right_tool:=yubi
# Model only:
ros2 launch detachable_tools hand_model.launch.xml side:=lhand tool:=yubi
```

ROS 2 uses `lhand_yubi_joint` / `rhand_yubi_joint` and each side's
`yubi_gripper_controller`. YUBI is registered in both SRDFs, so startup ID
detection and runtime tool selection also recognize `yubi`.
