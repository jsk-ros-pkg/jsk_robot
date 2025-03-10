# jsk_unitree_robot

ROS package for Unitree Go1 robot.

This document assumes that your robot is already configured with JSK's ROS environment. To configure this, please ask your robot's administrator along with [this instruction](./cross/README.md#setup-go1-robot).

## How to Run

See [manual](https://drive.google.com/drive/folders/1PZDOo8WUcqwU8mNek2qAaTwW9WjJVVRL?usp=sharing) before you use Go 1. (jsk.imi.i.u-tokyo.ac.jp account is required.)

### Setup Environment

First, you need to install ros. For ros melodic, please refer to install guide like [here](http://wiki.ros.org/melodic/Installation/Ubuntu)

```bash
mkdir -p catkin_ws/src
cd  catkin_ws/src
wstool init .
wstool set --git jsk-ros-pkg/jsk_robot https://github.com/jsk-ros-pkg/jsk_robot.git -y
wstool merge -t . https://raw.githubusercontent.com/jsk-ros-pkg/jsk_robot/master/jsk_unitree_robot/unitree.rosinstall
wstool update -t .
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build go1_description
catkin build unitreeeus jsk_unitree_startup
source devel/setup.bash
```

### Program Go1 robot

1. Connect to the robot via Ethernet

2. Configure your PC's ethernet like below

  - IP Address: 192.168.123.162
  - Subnet Mask: 255.255.255.0
  - Gateway: 192.168.123.161

3. Setup ROS_IP and ROS_MASTER_URI

   ```
   rossetmaster 192.168.96.161
   rossetip 192.168.96.162
   ```

4. Run roseus

   ```lisp
   (load "package://unitreeeus/unitree-interface.l") ;; load modules
   (go1-init)
   ```

   See `https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_unitree_robot/unitreeeus/test/test-go1.l` for example.

5. Deployment

   Once you have completed your development, put your code into [apps](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_unitree_robot/jsk_unitree_startup/apps/) directory and build on cross environment and copy to onboard computer.

   To run cross compile, you need `ros1-unitree` docker image and `arm64v8_System` directory. If you do not have them, pleas ask your development environment administrator along with following instructions.


   - [prepare-cross-compiling-environment-run-only-the-fist-time-per-host-computer](./cross/README.md#prepare-cross-compiling-environment-run-only-the-fist-time-per-host-computer)
   - [build-ros-system-on-docker--run-only-the-fist-time-per-host-computer](./cross/README.md#build-ros-system-on-docker--run-only-the-fist-time-per-host-computer)

   After that, please run command below to build ros workspace and deploy it to robot.

   ```
   roscd jsk_unitree_startup/../cross
   make user
   ./install -t Pro -p 123 # for Go1 Pro
   ./install -t Air -p 123 # for Go1 Air
   ```

   Then, reboot the robot and go to [app_chooser](http://192.168.123.161:8000/rwt_app_chooser) and start your application

## Lead teleop

[lead_teleop.launch](./jsk_unitree_startup/launch/lead_teleop.launch) is designed to interface with a mechanical button and an LED,
publishing joystick messages (`joy_msg`) to ROS using the rosserial library.
The node captures button presses and publishes the count of presses as part of the `/joy` messages to ROS.

### Hardware Setup

- Joystick: Interprets analog signals from two axes.
- LEDs: One Adafruit NeoPixel strip for status indication.
- Button: A single mechanical button used to trigger different commands based on the number of presses.

For detailed instructions on setting up the Arduino IDE, see [Setup Arduino IDE for LeadJoyDevelopment](#setup-arduino-ide-for-leadjoydevelopment).

### Setup Arduino IDE for LeadJoyDevelopment
- Add Seeduino XIAO to board manager
    - Open arduino IDE. `~/arduino-$ARDUINO_VERSION/arduino`
    - Add following Boards Manager URLs (File -> Preferences -> Additional Boards Manager URLs -> Click icon)
      - `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
    - Install `Seeed XIAO RP2040` by Seeed Studio (Tools -> Board -> Boards Manager -> Seeed XIAO RP2040)
    - Install `Adafruit NeoPixel` (>=1.10.2) by Adafruit (Tools -> Manage Libraries -> Adafruit NeoPixel)
    - Select`Seeed XIAO RP2040` Board (Tools -> Board -> Seeed RP2040 Boards -> Seeed XIAO RP2040)
    - Tested environment:
      - Arduino 1.8.19
      - Seeed XIAO RP2040: 1.12.0
      - Adafruit NeoPixel: 1.10.5

### ROS Parameters

- `~deadzone`: Adjusts the sensitivity of the joystick's zero point.
- `~press_interval`: Time window to register multiple button presses as a single action.
- `~debounce_period`: Debounce time to stabilize the button input.

### Visual Feedback

The Mechanical keyboard LED button changes colors based on the button pressed to provide immediate visual feedback. Each button press is associated with a different color.

### Button Press Actions and LED Feedback

| Button Presses | LED Color         | Action                        | Description                           |
|----------------|-------------------|-------------------------------|---------------------------------------|
| 0 presses       | Red | no action                           | Unitree is just standing.       |
| 1 press        | Blue | Sit                           | Unitree sits and will not walk.       |
| 2 presses      | Green | Stand                         | Unitree stands up and is able to walk.|
| 3 presses      | Yellow | no action                            | no action                                     |
| 4 presses      | White | Disable movement           | Unitree stops moving.                 |
| 5 presses      | Purple | Enable movement           | Unitree is able to move again.        |

The robot's motion and action mappings, such as stopping, moving, sitting, and standing, are defined in [rosserial_node.launch](./jsk_unitree_startup/launch/rosserial_node.launch) and [lead_joystick_teleop.yaml](./jsk_unitree_startup/config/lead_joystick_teleop.yaml). These files control the robot's responses to joystick inputs and can be customized as needed.

You can see it in the next video.

[lead_teleop](https://drive.google.com/file/d/1I7L-Ib8dmE77NPU4tOTtzfbOxjwT2nNe/view?usp=sharing)

## Topics

See [List of Topics](Go1_Topics.md) for list of topics used in Go1 robot.

## Tips

### Check boot process

#### 192.168.123.161

Auto start services list

```
$ cat ~/Unitree/autostart/.startlist.sh
updateDependencies
ipconfig
gencamparams
camerarosnode
03persontrack
imageai
jsk_startup
faceLightServer
slamDetector
wsaudio
faceLightMqtt
```

Monitor boot process

```
sshpass -p 123 ssh pi@192.168.123.161 tail -f Unitree/autostart/.startlog
```

#### 192.168.123.14

Auto start services list

```
$ cat ~/Unitree/autostart/.startlist.sh
updateDependencies
ipconfig
gencamparams
camerarosnode
03persontrack
imageai
jsk_startup
faceLightServer
slamDetector
wsaudio
faceLightMqtt
```

Monitor boot process

```
sshpass -p 123 ssh unitree@192.168.123.14 tail -f Unitree/autostart/.startlog
```

### Check roslaunch output

```
sshpass -p 123 ssh unitree@192.168.123.14 ls .ros/log/latest/
sshpass -p 123 ssh unitree@192.168.123.14 tail -f .ros/log/latest/app_manager-5.log
```

### Check AI Camera's output

```
mosquitto_sub -h 192.168.123.161 -t 'vision/human_pose' -d
```

### Stop auto started launch process

run `ps -auxwww` to find roslauch process and send `HUP`.

```
unitree@unitree-desktop:~$ ps -auxwww | grep roslaunch
unitree   7997  0.3  1.5 298308 62056 ?        Sl   16:24   0:05 /usr/bin/python /opt/ros/melodic/bin/roslaunch jsk_unitree_startup unitree_bringup.launch network:=ethernet
unitree   8553  0.0  0.0   7420   648 pts/2    S+   16:51   0:00 grep --color=auto roslaunch
unitree@unitree-desktop:~$ kill -HUP 7997
```

### Run ROS command on Go1 Comptuer

ROS system are installed under `/opt/jsk` direcotry. To enable ROS debugging onboard, type `source /opt/jsk/User/user_setup.bash`.

