## :set-body-pose-with-speed (naoqi_pose [`master`])

### What is this?

A robot takes a defined pose.  

### Location

`naoqi_pose/launch/pose_manager.launch`  

### NAOqi API

[ALRobotPosture::goToPosture](http://doc.aldebaran.com/2-4/naoqi/motion/alrobotposture-api.html#alrobotposture-api)

### Sample

```
send *ri* :set-body-pose-with-speed "Stand"
```