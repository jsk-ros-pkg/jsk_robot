## :get-external-collision-protection-status `type` (naoqi_bridge [`kochigami-develop`])

### What is this?

Check if the external collision protection is activated on the given name.  

### Parameters

`type`: body parts type of a robot (int)  

```
All:  0
Move: 1
Arms: 2
Larm: 3
Rarm: 4 
```
### Location

`naoqi_apps/launch/external_collision_avoidance.launch`  

### NAOqi API

[ALMotion::getExternalCollisionProtectionEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/reflexes-external-collision-api.html#ALMotionProxy::getExternalCollisionProtectionEnabled__ssCR)  

Related commit is [here](https://github.com/kochigami/naoqi_bridge/commit/7655dea24e26df3c0c2fae3fda20b6e96111d898#diff-e9b6c21fdccdb01cff79b583fc7ad7d2)

### Sample

```
; get status of external collision protection for Move part
send *ri* :get-external-collision-protection-status 2 
t
```
