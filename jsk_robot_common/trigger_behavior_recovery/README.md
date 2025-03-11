# Trigger Behavior Recovery

This recovery behavior will enables a robot to execute an action server behavior as a recovery behavior.
This will be useful when a robot stuck in a crowded environment.

<Image TODO>

## `trigger_behavior_recovery/TriggerBehaviorRecovery` plugin

### Parameters

- `~duration_timeout` (double, default: `10.0`)

Timeout duration for waiting for trigger behavior action server. (seconds)

- `~result_timeout` (double, default: `60.0`)

Timeout duration for trigger behavior action server. (seconds)

- `~trigger_action` (string, default: `trigger_default_behavior`)

Action name of an action server behavior.

### example

Please see demo.launch for example usage.

```bash
roslaunch trigger_behavior_recovery demo.launch
```

## Default trigger behavior servers

### speak_and_wait_behavior.py

Trigger behavior version of `speak_and_wait_recovery`.

#### Parameters

- `~speak_text` (string, default: `Hello World!`)

Text to be spoken.

- `~duration_wait` (double, default: `5.0`)

Duration of wating after speaking.

- `~sound_action` (string, default: `sound_play`)

Sound action server name.

#### Action server

- `~behavior` (type: `trigger_behavior_msgs/TriggerBehaviorAction`)

Trigger behavior action server.
