# How to setup a internal PC and spot user

This page describes how to setup a internal pc and spot user.

## How to setup a PC

### Install Ubuntu and ROS

TODO

### Setup wifi interfaces

TODO


## How to set up the spot user

First, create a `spot` user to your PC if it does not exist.

```
TODO
```

And add `spot` user to sudo group.

```
sudo gpasswd -a spot sudo
```

systemd services of JSK Spot system use workspaces in `spot` user.
`spot` user should have 3 workspaces for this use.

- `spot_driver_ws`: a workspace to run driver.launch. which requires python3 build version of geometry3
- `spot_coral_ws`: a workspace to run object_detection.launch ( which includes coral_usb_ros node ) which requires python3 build version of opencv_brindge
- `spot_ws`: a workspace to run other launch ( python2 )

### Prerequities

Install necessary packages for workspace building

```
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install ros-melodic-catkin
```


### setting up `spot_driver_ws`

Create a workspace for `spot_driver`.

```bash
source /opt/ros/melodic/setup.bash
mkdir ~/spot_driver_ws/src -p
cd ~/spot_driver_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_driver.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd ~/spot_driver_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build -j4 -c
```

After this, please modify the credential files for spot_driver.

```bash
roscd jsk_spot_startup
# modify auth/credential_config.yaml
git update-index --skip-worktree auth/spot_credential.yaml
```


### setting up `spot_coral_ws`

First, follow [Edge TPU dependencies installation section of coral_usb_ros](https://github.com/knorth55/coral_usb_ros#edge-tpu-dependencies-installation)
Then, create a workspace for coral_usb.

```bash
source /opt/ros/melodic/setup.bash
mkdir ~/spot_coral_ws/src -p
cd ~/spot_coral_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool set coral_usb_ros https://github.com/knorth55/coral_usb_ros.git --git
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_coral.rosinstall
wstool merge -t . coral_usb_ros/fc.rosinstall.melodic
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
cd ~/spot_coral_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build -j4 -c
```

After that, please download models for coral_usb_ros.

```
source /opt/ros/melodic/setup.bash
source ~/spot_coral_ws/devel/setup.bash
rosrun coral_usb download_models.py
```


### setting up `spot_ws`

Create `spot_ws`

```bash
source /opt/ros/melodic/setup.bash
mkdir ~/spot_ws/src -p
cd ~/spot_ws/src
wstool init .
wstool set jsk-ros-pkg/jsk_robot https://github.com/sktometometo/jsk_robot.git --git -v develop/spot
wstool update
wstool merge -t . jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_dev.rosinstall
wstool update
rosdep update
rosdep install -y -r --from-paths . --ignore-src
pip3 install -r jsk-ros-pkg/jsk_robot/jsk_spot_robot/requirements.txt
cd ~/spot_ws
catkin init
catkin build -j4 -c
```

If you want to use switchbot_ros with spot_basic_behaviors, please add switch_bot token.

```
roscd spot_basic_behaviors
# modify config/switchbot_ros/token.yaml
git update-index --skip-worktree config/switchbot_ros/token.yaml
```


### Setup Google Service Clients

In order to use `dialogflow_task_executive` and `gdrive_ros`, please prepair authentication files.

- [dialogflow_task_executive](https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/dialogflow_task_executive/README.md) 
  + Please download a service account key JSON file of your dialogflow project and put it under `/var/lib/robot/`.
- [gdrive_ros](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/gdrive_ros)
  + Please download client config file, create `settings.yaml` and put them under `/var/lib/robot/`.


### Install scripts and systemd services

Install scripts and systemd services

```
source /opt/ros/melodic/setup.bash
source ~/spot_ws/devel/setup.bash
rosrun jsk_spot_startup deploy-scripts-and-services.sh
```


### Set environmental variables

Modify `/var/lib/robot/config.bash` to set environmental variables.

Set `IF_ETH`, `IF_WIFI`, `IF_LTE` to your network interface names to use `jsk-spot-utils-network.service`

```
export IF_ETH="noethernetdevice"
export IF_WIFI="nowifidevice"
export IF_LTE="noltedevice"
```

Set your `ROS_IP` to WiFi AP address to ip address of your wifi AP interface.

```
WIFI_AP_IP=10.42.0.1
rossetmaster $WIFI_AP_IP
rossetip $WIFI_AP_IP
```

Add environmental variables for [dialogflow_task_executive](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/dialogflow_task_executive) and [gdrive_ros](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/gdrive_ros)
Please see them for more details.

```
export GOOGLE_APPLICATION_CREDENTIALS=/path/to/service_account_json_file # for dialogflow
export DIALOGFLOW_PROJECT_ID=<your dialogflow project id> # for dialogflow
export GOOGLE_DRIVE_SETTINGS_YAML=/path/to/pyDrive_setting_yaml # for pydrive
```


### Setting up udev files

Create udev rule for insta360air, spot spinal.

```bash
roscd jsk_spot_startup
sudo cp udev_rules/* /etc/udev/rules.d/
```

Create udev rules for joy pads

```bash
roscd jsk_spot_teleop
sudo cp config/udev/* /etc/udev/rules.d/
```

And reload them

```bash
sudo udevadm control --reload-rules
```

Please modify each udev rule according to your configuration.


### Add the user to groups

Add spot user to groups

```bash
sudo gpasswd -a <your user> dialout
sudo gpasswd -a <your user> audio
sudo gpasswd -a <your user> plugdev
sudo gpasswd -a <your user> video
```

#### Setup cockpit

Cockpit is server management tool. If you use Spot CORE cockpit is already setup.

```
sudo apt install cockpit
```

```
roscd jsk_spot_startup
sudo cp -r config/cockpit.socket.d /etc/systemd/system/cockpit.socket.d
sudo systemctl daemon-reload
sudo systemctl start cockpit.socket
```

Then you can access cockpit by `https://<robot ip>:21443`

#### Setup postfix with gmail

TODO
