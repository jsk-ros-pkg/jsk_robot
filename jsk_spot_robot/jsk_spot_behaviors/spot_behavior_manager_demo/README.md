# spot_behavior_manager_demo

This package is a demo package of spot_behavior_manager, which includes

- A demo node of spot_behavior_manager node
- Sample config map file for spot_behavior_manager
- A demo launch
- Utility scripts

## scripts

### visualize_map.py

Visualizer script of map file of spot_behavior_manager

Befor using this script, you need to install

```
pip3 install graphviz xdot
```

#### Usage

```
rosrun spot_behavior_manager_demo visualize_map.py --filename <config map yaml>
```
