## :servo-off (naoqi_bridge [`master`])

### What is this?

A robot sets motor off and takes a reset pose.  

### Location

`naoqi_pose/launch/pose_manager.launch`  

### NAOqi API

[ALMotionProxy::rest](http://doc.aldebaran.com/2-5/naoqi/motion/control-stiffness-api.html#ALMotionProxy::rest)

### Sample

```
send *ri* :servo-off
```
