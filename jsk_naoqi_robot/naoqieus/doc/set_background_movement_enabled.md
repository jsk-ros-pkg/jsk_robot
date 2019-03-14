## :set-background-movement-enabled `status` (naoqi_bridge [`kochigami-develop`])

### What is this?

Enable or disable the background movements. For further details on background movement, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albackgroundmovement.html#albackgroundmovement).   

### Parameters

`status`: t/ nil (bool)  

### Location

`naoqi_apps/launch/background_movement.launch`  

### NAOqi API

[ALBackgroundMovement::setEnabled](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albackgroundmovement-api.html#ALBackgroundMovementProxy::setEnabled__b)  

Related commit is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/82)

### Sample

```
; enable background movement
send *ri* :set-background-movement-enabled t
```
