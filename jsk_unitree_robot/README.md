# jsk_unitree_robot

ROS package for Unitree Go1 robot.

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

2. Set your PC'S IP address to `192.168.96.162`

3. Setup ROS_IP and ROS_MASTER_URI

   ```
   rossetmaster 192.168.96.161
   rossetip
   ```

4. Run roseus

   ```lisp
   (load "package://unitreeeus/unitree-interface.l") ;; load modules
   (go1-init)
   ```

   See `https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_unitree_robot/unitreeeus/test/test-go1.l` for example.

5. Deployment

   Once you have completed your development, put your code into [apps](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_unitree_robot/jsk_unitree_startup/apps/) directory and build on cross environment and copy to onboard computer.
   ```
   roscd jsk_unitree_startup/../cross
   make user
   ./install -p 123
   ```

   Then, reboot the robot and go to [app_chooser](http://192.168.123.161:8000/rwt_app_chooser) and start your application

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
      - Seeed XIAO RP2040: 1.9.3
      - Adafruit NeoPixel: 1.10.5
 - Make rosserial_arduino libraries
    ```bash
    source ~/catkin_ws/devel/setup.bash
    cd ~/arduino-$ARDUINO_VERSION/libraries
    rm -rf ros_lib
    rosrun rosserial_arduino make_libraries.py .
    ```

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

