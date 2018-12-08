## :get-background-movement-enabled (naoqi_bridge [`kochigami-develop`])

### What is this?

Return whether the background movements are enabled. For further details on background movement, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albackgroundmovement.html#albackgroundmovement).

### Location

`naoqi_apps/launch/background_movement.launch`  

### NAOqi API

[ALBackgroundMovement::isEnabled](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albackgroundmovement-api.html#ALBackgroundMovementProxy::isEnabled)  

Related PR is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/82)

### Sample

```
; get background movement status
send *ri* :get-background-movement-enabled
t
```
