# update_move_base_parameter_recovery

![update_move_base_parameter_recovery_resized](https://user-images.githubusercontent.com/9410362/189819463-f75acdd5-4645-402a-9b8d-a1477553e028.gif)

This package provides recovery behaviors which update dynamic parameter of move_base with dynamic reconfigure.

Currently there are plugins below in this package.

- update_move_base_parameter_recovery/UpdateInflationLayerParameterRecovery
- update_move_base_parameter_recovery/UpdateCostmapParameterRecovery

To run a demo

```bash
roslaunch update_move_base_parameter_recovery demo.launch
```

## update_move_base_parameter_recovery/UpdateInflationLayerParameterRecovery

A recovery behavior plugin which updates `InflationLayerPlugin` dynamic parameters.

### Parameters

- `~parameter_name` (type: `string`, default: ``)

Dynamic parameter name which the plugin updates. (e.g. `/move_base_node/local_costmap/inflation_layer`)

- `~duration_deadline` (type: `float`, default: `10.0`)

Duration in which original parameters are restored.

- `~timeout_duration` (type: `float`, default: `5.0`)

Timeout duration for dynamic reconfigure client.

### Parameters to update

These paremeters are only used when specified. If not specified, original parameters of the target `InflationLayerPlugin` is used.

- `~enabled` (type: `bool`)

- `~cost_scaling_factor` (type: `float`)

- `~inflation_radius` (type: `float`)

- `~inflate_unknown` (type: `bool`)

## update_move_base_parameter_recovery/UpdateCostmapParameterRecovery

A recovery behavior plugin which updates `Costmap2D` dynamic parameters.

### Parameters

- `~parameter_name` (type: `string`, default: ``)

Dynamic parameter name which the plugin updates. (e.g. `/move_base_node/local_costmap`)

- `~duration_deadline` (type: `float`, default: `10.0`)

Duration in which original parameters are restored.

- `~timeout_duration` (type: `float`, default: `5.0`)

Timeout duration for dynamic reconfigure client.

### Parameters to update

These paremeters are only used when specified. If not specified, original parameters of the target `Costmap2D` is used.

- `~footprint` (type: `string`)

- `~robot_radius` (type: `float`)

- `~footprint_padding` (type: `float`)
