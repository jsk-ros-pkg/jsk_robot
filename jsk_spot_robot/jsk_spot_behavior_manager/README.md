# jsk_spot_behaviors

These packages enable for Spot to execute locomotoion behaviors to reach a desired position.

## Concept

In this framework, knowledge about positions and transtion behaviors between them are represented as a digraph like below.

Each node represents specified positions and each edge represents a behavior to transition between them.

![example_graph](https://user-images.githubusercontent.com/9410362/124147589-cc8ce700-dac9-11eb-930f-1c00c2a4777e.png)

A graph is defined by a yaml file ( e.g. [node.yaml in spot_behavior_graph](./spot_behavior_graph/config/node.yaml) and [edge.yaml in spot_behavior_graph](./spot_behavior_graph/config/edge.yaml)  )
Please see nodes and edges format section below.

Knowledge representation and execution process of behaviors are separated from actual behavior implementation.
The implementation of former is in spot_behavior_manager and spot_behavior_manager_server, but actual behaviors like walk and elevator are in spot_basic_behaviors.
behavior_manager_server node will dynamically load each behaviors defined in map.yaml so you can easilly add your behavior without editing these core implementation.
Please see spot_basic_behaviors package for behavior examples.

## How to use it

To run demo, please make Spot stand in front of the fiducial in 73B2 and run

```bash
roslaunch spot_behavior_graph demo.launch
```

```bash
$ rostopic pub -1 /spot_behavior_manager_server/lead_person/execute_behaviors spot_behavior_manager_msgs/LeadPersonActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_node_id: 'eng2_2FElevator'" 
```

Then Spot will go to 2FElevator of eng2 by walk behavior and elevator behavior implemented in spot_basic_behaviors packages.

https://user-images.githubusercontent.com/9410362/124338016-aad25380-dbe0-11eb-962f-b9a27e1e08cb.mp4

For more details, please see [spot_behavior_manager](./spot_behavior_manager), [spot_behavior_manager_server](./spot_behavior_manager_server) and each behavior documentation. (e.g. [spot_basic_behaviors](./spot_basic_behaviors) )

## Behavior Graph


This package provides graph and node data for spot_behavior_manager.

### scripts

#### visualize_map.py

Visualizer script of map file of spot_behavior_manager

Befor using this script, you need to install

```
pip3 install graphviz xdot
```

##### Usage

```
rosrun spot_behavior_graph visualize_map.py --filename <config map yaml>
```

##### Example Output

![map](https://user-images.githubusercontent.com/9410362/132942120-4a4e652b-3d25-43df-a678-fd3c09782284.png)

## behavior manager server

This package provides ros nodes for spot_behavior_manager.

### ROS Node

#### behavior_manager_server.py

ROS Node script for spot_behavior_manager.

See [the source of behavior_manager_node](../spot_behavior_manager/src/spot_behavior_manager/behavior_manager_node.py).

#### interactive_behavior_executor.py

ROS Node script to execute action of spot_behavior_manager by language.

