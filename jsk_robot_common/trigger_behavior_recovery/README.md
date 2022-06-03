# Trigger Behavior Recovery

This recovery behavior will enables a robot to execute an action server behavior as a recovery behavior.
This will be useful when a robot stuck in a crowded environment.

<Image TODO>

## trigger_behavior_recovery/TriggerBehaviorRecovery

### Parameters

- `~duration_timeout` (double, default: `1.0`)

Duration for tf looking up transformations. (seconds)

- `~trigger_behavior_action` (string, default: `trigger_default_action`)

Action name of an action server behavior.

### example

Please see demo.launch for example usage.

```bash
roslaunch trigger_behavior_recovery demo.launch
```
