## :set-external-collision-protection-status `type` `status` (naoqi_bridge [`kochigami-develop`])

### What is this?

Enable/disable external collision protection of a robot on the given name.  

### Parameters

`type`: body parts type of a robot (int)  

```
All:  0
Move: 1
Arms: 2
Larm: 3
Rarm: 4 
```

`status`: t/ nil (bool)  

### Location

`naoqi_apps/launch/external_collision_avoidance.launch`  

### NAOqi API

[ALMotion::setExternalCollisionProtectionEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/reflexes-external-collision-api.html#ALMotionProxy::setExternalCollisionProtectionEnabled__ssCR.bCR)  

Related commit is [here](https://github.com/kochigami/naoqi_bridge/commit/7655dea24e26df3c0c2fae3fda20b6e96111d898#diff-e9b6c21fdccdb01cff79b583fc7ad7d2)

### Sample

```
; disable external collision protection for Move part
send *ri* :set-external-collision-protection-status 1 nil
```
