## :set-body-pose-with-speed (naoqi_pose [`master`])

### What is this?

Returns defined pose list which can be available in `:set-body-pose-with-speed` method. 

### Location

`naoqi_pose/launch/pose_manager.launch`  

### NAOqi API

[ALRobotPosture::getPostureList](http://doc.aldebaran.com/2-4/naoqi/motion/alrobotposture-api.html#alrobotposture-api)

### Sample

```
send *ri* :get-defined-pose-list
"Crouch LyingBack LyingBelly Sit SitOnChair SitRelax Stand StandInit StandZero"
```