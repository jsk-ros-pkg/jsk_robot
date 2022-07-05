# softhand_ros

ROS package for SoftHand

## Note

This package depends on the branch below.

- [pazeshun/dynamixel_motor@gripper-v6-devel](https://github.com/pazeshun/dynamixel_motor/tree/gripper-v6-devel)

## Installation

### Workspace build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir ~/softhand_ws/src -p
cd ~/softhand_ws/src
wget https://raw.githubusercontent.com/knorth55/softhand_ros/master/fc.rosinstall -O .rosinstall
wstool up
rosdep install --ignore-src --from-paths . -y -r -i
cd ~/softhand_ws
catkin build
```

### Udev installation

```bash
source ~/softhand_ws/devel/setup.bash
roscd softhand_ros
sudo cp udev/*.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
```

## How to use

### Launch softhand

#### For softhand v1

```bash
source ~/softhand_ws/devel/setup.bash
# for left softhand v1
roslaunch softhand_ros softhand_left.launch
# for right softhand v1
roslaunch softhand_ros softhand_right.launch
```

#### For softhand v2

```bash
source ~/softhand_ws/devel/setup.bash
# for left softhand v2
roslaunch softhand_ros softhand_v2_left.launch
# for right softhand v2
roslaunch softhand_ros softhand_v2_right.launch
```

### Control softhand by euslisp

#### For softhand v1

```bash
source ~/softhand_ws/devel/setup.bash
roscd softhand_ros/euslisp
roseus softhand-interface.l
# euslisp interactive mode
# (softhand-init)
# (send *ri* :start-grasp)
# (send *ri* :stop-grasp)
```

#### For softhand v2

```bash
source ~/softhand_ws/devel/setup.bash
roscd softhand_ros/euslisp
roseus softhand-v2-interface.l
# euslisp interactive mode
# (softhand-v2-init)
# (send *ri* :close-thumb)
# (send *ri* :open-thumb)
# (send *ri* :start-grasp)
# (send *ri* :stop-grasp)
```

## Softhand hardware installation

### Set baud rate

#### For softhand v1

```bash
# set baud rate from 57600 to 1000000
rosrun dynamixel_driver set_servo_config.py -b 57600 -r 1 MOTOR_ID
```

#### For softhand v2

```bash
# set baud rate from 57600 to 57143
rosrun dynamixel_driver set_servo_config.py -b 57600 -r 34 MOTOR_ID
```

### Set motor ID

```bash
rosrun dynamixel_driver change_id.py OLD_MOTOR_ID NEW_MOTOR_ID
```

#### Motor IDs of Softhand v1

- 1: Thumb
- 2: Index finger
- 3: Middle finger

#### Motor IDs of Softhand v2

- 1: Thumb rotate
- 2: Thumb
- 3: Index finger
- 4: Middle finger

### Disable overload error

#### For softhand v1

```python
import roslib
roslib.load_manifest('dynamixel_driver')
from dynamixel_driver import dynamixel_io

# for softhand v1
dxl_io = dynamixel_io.DynamixelIO("/dev/ttyUSB0", 1000000)
dxl_io.write(MOTOR_ID, 17, (4,))
dxl_io.write(MOTOR_ID, 18, (4,))
```

#### For softhand v2

```python
import roslib
roslib.load_manifest('dynamixel_driver')
from dynamixel_driver import dynamixel_io

# for softhand v2
dxl_io = dynamixel_io.DynamixelIO("/dev/ttyUSB0", 57600)
dxl_io.write(MOTOR_ID, 17, (4,))
dxl_io.write(MOTOR_ID, 18, (4,))
```

#### Change `product` to distinguish E151 board

We distinguish left and right hand with `product` field of FTDI chip on E151.

- Left softhand v1 E151's `product`: `LEFT-E151`
- Right softhand v1 E151's `product`: `RIGHT-E151`
- Left softhand v2 E151's `product`: `LEFT-V2-E151`
- Right softhand v2 E151's `product`: `RIGHT-V2-E151`

In order to change them, please follow [jsk_apc doc](https://jsk-apc.readthedocs.io/en/latest/jsk_arc2017_baxter/setup_gripper_v6.html#distinguish-left-dxhub-from-right-one).

If you don't have windows, you can do it with [richardeoin/ftx-prog](https://github.com/richardeoin/ftx-prog) as follows.

```bash
git clone https://github.com/richardeoin/ftx-prog.git
cd ftx-prog/
make
# for left softhand v1
sudo ./ftx-prog --product LEFT-E151
# for right softhand v1
sudo ./ftx-prog --product RIGHT-E151
# for left softhand v2
sudo ./ftx-prog --product LEFT-V2-E151
# for right softhand v2
sudo ./ftx-prog --product RIGHT-V2-E151
```
