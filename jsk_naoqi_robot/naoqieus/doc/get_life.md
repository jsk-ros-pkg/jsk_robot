## :get-life (naoqi_bridge [`master`])

### What is this?

Return AutonomousLife state. ("solitary", "interactive", "safeguard", "disabled")

### Location

`naoqi_pose/launch/pose_manager.launch`  

### NAOqi API

[ALAutonomousLife::getState](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomouslife-api.html#ALAutonomousLifeProxy::getState)  
For further details of AutonomousLife state, please refer to [here](http://doc.aldebaran.com/2-5/ref/life/state_machine_management.html#autonomouslife-states).

### Sample

```
send *ri* :get-life
"disabled"
```
