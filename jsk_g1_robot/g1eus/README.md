# g1eus

![g1eus](./figs/g1eus.png)

`g1eus` is the ROS 2 package providing a roseus interface and EusLisp model for the Unitree G1 robot.


## Setup

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
wget https://raw.githubusercontent.com/Michi-Tsubaki/jsk_robot/refs/heads/ros2/jsk_g1_robot/g1eus/jazzy.repos -O g1eus.jazzy.repos
vcs import < g1eus.jazzy.repos
source /opt/ros/jazzy/setup.bash
cd ~/colcon_ws
sudo apt update
rosdep update
rosdep install -iqry --from-paths src --ignore-src
colcon build --symlink-install --packages-up-to g1eus g1_bringup
source install/setup.bash
```

## Kinematic Simulator

The interface can run without a real robot by using the kinematic simulator.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
roseus
```

```lisp
(load "package://g1eus/g1-interface.l")
(g1-init)
(send *ri* :angle-vector (send *g1* :angle-vector) 1000)
(send *ri* :wait-interpolation)
```

## Real Robot

This interface supports the normal `g1_bringup` mode and does not support GEART SONIC mode.

Do not use `use_gear_sonic:=true`.

```bash
ssh unitree@192.168.123.164
sudo systemctl stop ros2-g1-gear-sonic-bringup.service gear-sonic.service
sudo systemctl status ros2-g1-gear-sonic-bringup.service gear-sonic.service --no-pager
```

For default ROS 2 mode, do not start `ros2-g1-gear-sonic-bringup.service`.

If you use Inspire RH56DFX hands, keep `inspire-g1.service` running, or start the hand service manually when systemd is not installed.

```bash
sudo systemctl start inspire-g1.service
sudo systemctl status inspire-g1.service --no-pager
```

Manual hand-service startup is as follows.

```bash
ssh unitree@192.168.123.164
cd ~/dfx_inspire_service/build
sudo ./inspire_g1 -k -u
```

On the ROS 2 computer, connect Ethernet to G1, set the network interface as described in the Unitree G1 developer instructions and launch the default bringup.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network_interface>
```


With Inspire RH56DFX hands,

```bash
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network_interface> hand_type:=inspire_dfq
```


Then activate the upper-body trajectory controller.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 control set_controller_state upper_body_controller active
```


### Run roseus

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
roseus
```

```lisp
(load "package://g1eus/g1-interface.l")
(g1-init)
(send *ri* :angle-vector (send *g1* :angle-vector) 3000)
(send *ri* :wait-interpolation)
```


### Command Velocity

In the default ROS 2 mode, `g1_hardware` starts `loco_cmd_adapter`, which subscribes to `/cmd_vel`.
`g1-interface.l` publishes `geometry_msgs/Twist` to `/cmd_vel` by default.

```lisp
(send *ri* :cmd-vel 0.1 0.0 0.0)  ;; x, y, yaw
(send *ri* :cmd-vel 0.0 0.1 0.0)
(send *ri* :cmd-vel 0.0 0.0 0.2)
(send *ri* :stop-base)
```


## Hand Interface

```lisp
(send *ri* :hand-angle-vector :larm #f(0 0 0 0 0 0) 1000)

(send *ri* :stop-grasp :larm)
(send *ri* :prepare-grasp :larm)
(send *ri* :start-grasp :larm)
(send *ri* :default-grasp :larm)
```

The right hand uses `:rarm` in the same way.
