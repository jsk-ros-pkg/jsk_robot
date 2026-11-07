# detachable_tools

Detachable surgical tools (gripper, needle holder, forceps) driven by Dynamixel motors, for ROS 1 and ROS 2.

| Tool | Hardware | Model |
| --- | --- | --- |
| `gripper` | <img src="figs/real_hand.jpg" width="200"> | <img src="figs/sim_gripper_oblique.png" width="260"> |
| `needle_holder` | <img src="figs/real_holder.jpg" width="200"> | <img src="figs/sim_needle_holder_oblique.png" width="260"> |

## Setup

```bash
# ROS 1
cd <catkin workspace>
vcs import src < src/jsk-ros-pkg/jsk_robot/jsk_hand/detachable_tools/ros1.yaml
rosdep install -iry --from-paths src
catkin build detachable_tools

# ROS 2
cd <colcon workspace>
vcs import src < src/jsk-ros-pkg/jsk_robot/jsk_hand/detachable_tools/ros2.yaml
rosdep install -iry --from-paths src
colcon build --packages-up-to detachable_tools
```

To use `/dev/lhand` and `/dev/rhand`:

```bash
sudo scripts/create_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

| Motor | Left ID | Right ID |
| --- | ---: | ---: |
| Detacher | 0 | 1 |
| Gripper (`gripper`, `generic`) | 2 | 3 |
| Needle holder (`needle_holder`) | 4 | 5 |
| Forceps (`forceps`) | 6 | 7 |
| YUBI (`yubi`) | 8 | 9 |

YUBI setup, source CAD, and model limitations: [YUBI](meshes/yubi/README.md).

## Usage

Start the hands (the launch files and arguments are the same on ROS 1, `roslaunch ... <name>.launch`):

```bash
ros2 launch detachable_tools hand_control.launch.xml side:=lhand tool:=gripper
ros2 launch detachable_tools dual_hand.launch.xml left_tool:=gripper right_tool:=needle_holder
ros2 launch detachable_tools hand_detacher.launch.xml side:=lhand            # no tool attached
ros2 launch detachable_tools hand_model.launch.xml side:=rhand tool:=needle_holder  # model only
```

### Automatically detect attached tools (ROS 1 / ROS 2)

```bash
ros2 launch detachable_tools dual_hand_auto.launch.xml
# ROS 1
roslaunch detachable_tools dual_hand_auto.launch
```

Automatically detects tools on `/dev/lhand` and `/dev/rhand` at startup.
Stop existing hand launches first. Restart this launch to detect tools again,
or use `set_tool/<tool>` to select a tool at runtime.

- For tool-only hardware, set `include_detacher:=false` (default: `true`).
  If only a tool responds, that side starts with the detacher disabled.
- To change serial settings, use `left_port_name`, `right_port_name`, and `baud_rate`.

### Inspect detected and selected tools

Startup console logs show the arm (`lhand`/left or `rhand`/right), serial port,
responding IDs, tool type, and whether a tool was detected.
Both ROS versions publish `diagnostic_msgs/DiagnosticArray` on:

| Topic (replace `lhand` with `rhand` for the right arm) | Contents |
| --- | --- |
| `/lhand/detected_tool` | Startup scan: side, port, detected IDs, tool, attached, `detection_scope=startup`. Auto launch only. |
| `/lhand/tool_state` | Selected model: side, tool, attached, motor IDs; updated on tool changes. |
| `/lhand/hand_description` | Selected model and individual motor specifications. |

Detection is a startup snapshot, not a live connection or controller-health
monitor. `tool_state` describes the selected model, not proof of successful
motor communication. Check controller_manager for controller/hardware status.
ROS 1 publishers are latched; ROS 2 publishers use reliable transient-local QoS
so late subscribers can retrieve the last message:

```bash
ros2 topic echo /lhand/detected_tool --qos-durability transient_local --qos-reliability reliable --once
ros2 topic echo /rhand/tool_state --qos-durability transient_local --qos-reliability reliable --once
# ROS 1
rostopic echo -n 1 /lhand/detected_tool
rostopic echo -n 1 /rhand/tool_state
```

### Python

```python
from hand_controller import LHandController, RHandController

lhand = LHandController()
lhand.detach()                 # release the tool
lhand.attach()                 # lock the tool
lhand.open()                   # gripper
lhand.close(effort=0.5)        # effort: grasp current [Nm]
lhand.open_holder()            # needle holder
lhand.close_holder()
lhand.open_forceps()           # forceps (also open_scissors / close_scissors)
lhand.close_forceps()
lhand.move_named("open", "needle_holder")  # any <group_state> of srdf/<side>.srdf
```

The open / close positions are the `<group_state>`s of `srdf/lhand.srdf` and `srdf/rhand.srdf`.

Switch the tool without restarting the launch (`generic`, `gripper`, `needle_holder`, `forceps`, `detacher`):

```bash
ros2 service call /lhand/set_tool/needle_holder std_srvs/srv/Trigger
ros2 service call /lhand/get_tool_state std_srvs/srv/Trigger
```

From the command line:

```bash
ros2 run detachable_tools hand_command.py --side lhand close
ros2 run detachable_tools hand_command.py --side lhand detach
```

### EusLisp

The same operations are available from EusLisp:

```lisp
(load "package://detachable_tools/euslisp/all.l")
(lhand-init)
(send *lhand* :close)
(send *lhand* :open-holder)
(send *lhand* :move-named "open" :tool "needle_holder")
(send *lhand* :detach :tm 1.0)
```

`euslisp/tool-change.l` switches the robot model, the IK target and the running hand together:

```lisp
(tool-setup-robot *nextage* :robot-kind :nextage)
(attach-tool *nextage* :rarm :needle-holder :physical t :restart-control t)
(tool-ik *nextage* :rarm target-coords :rotation-axis t)
(detach-tool *nextage* :rarm :physical t :restart-control t)
```

## Add new end-effectors

For a tool named `<tool>` (on both `lhand` and `rhand`):

1. Meshes: put them in `meshes/<tool>/` and refer to them as
   `package://detachable_tools/meshes/<tool>/<file>`.
2. URDF: add `urdf/<side>_<tool>.urdf`, starting from `urdf/<side>_generic.urdf`.
   - The root link is `<side>_base_link` and every name starts with `<side>_`.
   - `<side>_joint` is the tool motor (motor angle in rad, `revolute` with limits wider than
     the SRDF positions), with its `<transmission>` (actuator `<side>_motor`).
   - `<ros2_control name="<side>_<tool>">` has the motor `ID` (unique on the bus) and
     `Operating Mode` (5: current-based position) in `<gpio name="<side>_motor">`;
     rename the `generic/...` service names to `<tool>/...`.
3. SRDF: in `srdf/<side>.srdf`, add `<group name="<side>_<tool>">` with `<side>_joint`
   and its `<group_state>`s (at least `open` and `close`, motor angle in rad).
4. Joints that only follow the motor for display (fingers, linkages): add
   `config/<side>/<tool>_loop_couplings.yaml` (polynomial of `<side>_joint`).
5. ROS 2: in `config/<side>/controllers.yaml`, add `<side>_<tool>` to
   `hardware_components_initial_state.unconfigured` and a `<tool>_gripper_controller`
   (copy an existing one, joint `<side>_<tool>_joint`).

Then `set_tool/<tool>` and `move_named("open", "<tool>")` work for the new tool.

### Fork simulation (ROS 2 Jazzy)

Run in an environment where `nextage_tutorials` and `nextage_description` have been built.

```bash
sudo apt install ros-jazzy-apriltag-ros python3-scipy python3-opencv
colcon build --packages-select detachable_tools
source install/setup.bash
ros2 run detachable_tools apriltag_fork_sim.py
```

- `--config FILE`: Use a custom configuration (default: `config/fork_simulation.yaml`)
- `--no-rviz --once`: Run once without a GUI
- `--output DIR`: Directory for logs (default: `/tmp/apriltag_fork_sim/`)

### Example for Nextage fork (ROS 2)

Build `nextage_tutorials` and publish joint states, camera TFs, and detected AprilTag TFs before calling the services from a pose where the tag is visible.

```bash
ros2 run detachable_tools nextage_fork_server.py \
  --config "$(ros2 pkg prefix detachable_tools)/share/detachable_tools/config/nextage_fork_api.yaml"
```

Services use `std_srvs/Trigger` under `/fork_exchange/`, with JSON in the response `message` field.

- `plan_detach` / `plan_attach`: Check the planned paths
- `detach` / `attach`: Detach / attach the fork

Python:

```python
import rclpy
from detachable_tools_common.fork_api import NextageForkAPI

rclpy.init()
node = rclpy.create_node('fork_client')
fork = NextageForkAPI(node)
future = fork.plan_detach()  # The same applies to plan_attach(), detach(), and attach()
rclpy.spin_until_future_complete(node, future)
print(fork.result(future.result()))
node.destroy_node()
rclpy.shutdown()
```

EusLisp:

```lisp
(load "package://detachable_tools/euslisp/fork-exchange.l")
(nextage-fork-plan-detach)
(nextage-fork-plan-attach)
(nextage-fork-detach)
(nextage-fork-attach)
```

For real hardware, copy the configuration file and adjust the tag and tool transforms, camera TFs, observation poses, insertion direction, detach/attach currents and durations, and controller names to match your robot. Set `calibrated: true` and launch with `--hardware`.
