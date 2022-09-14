# complex_recovery

This package provides recovery behavior plugins which combines multi recoveries to one recovery behavior.
This is useful for assuring a set of recovery behavior to run at one time.

There are two types of recoveries. One is to run multi recoveries sequentially and another is to run them in parallel.

![complex_recovery_diagrams](https://user-images.githubusercontent.com/9410362/189815175-a3265d23-01d4-4ae9-831c-b01aacad2872.png)

## complex_recovery/SequentialComplexRecovery

* `~recovery_behaviors` (list of dictionaries which has `name` and `type` entries, default: None)

Example configuration of `move_base` is like

```yaml
recovery_behavior_enabled: true
recovery_behaviors:
  - name: "speak_then_clear_costmap0"
    type: "complex_recovery/SequentialComplexRecovery"
speak_then_clear_costmap0:
  recovery_behaviors:
    - name: "speak_and_wait0"
      type: "speak_and_wait_recovery/SpeakAndWaitRecovery"
    - name: "clear_costmap0"
      type: "clear_costmap_recovery/ClearCostmapRecovery"
  speak_and_wait0:
    speak_text: "I'm clearing costmap."
    duration_wait: 5.0
    duration_timeout: 1.0
    sound_action: /sound_play
  clear_costmap0:
    reset_distance: 1.0
```

In this case, `speak_and_clear_costmap0` recovery runs `speak_and_wait0` recovery first, then runs `clear_costmap0`.
So a robot speaks first and then clear its costmap.

## complex_recovery/ParallelComplexRecovery

* `~recovery_behaviors` (list of dictionaries which has `name` and `type` entries, default: None)

Example configuration of `move_base` is like

```yaml
recovery_behavior_enabled: true
recovery_behaviors:
  - name: "speak_and_rotate_costmap0"
    type: "complex_recovery/SequentialComplexRecovery"
speak_and_rotate_costmap0:
  recovery_behaviors:
    - name: "speak_and_wait0"
      type: "speak_and_wait_recovery/SpeakAndWaitRecovery"
    - name: "rotate0"
      type: "rotate_recovery/RotateRecovery"
  speak_and_wait0:
    speak_text: "I'm rotating."
    duration_wait: 5.0
    duration_timeout: 1.0
    sound_action: /sound_play
  rotate0:
    sim_granularity: 0.017
    frequency: 20.0
```

In this case, `speak_and_rotate_costmap0` recovery runs `speak_and_wait0` and `rotate0` simultaneously.
So a robot speaks during its rotation.
