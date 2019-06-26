## :start-grasp `&optional (angle-ratio 0.0) (arm :arms)` (naoqi_bridge [`master`])

### What is this?

Start grasping.

### Parameters

`angle-ratio`: ratio of grasping (float, 0.5-1.0 (default 0.0))  
`arm`: grasping arm type (str, `:larm`, `:rarm`, `:arms` (default `:arms`))  

### Location

`naoqi_pose/launch/pose_manager.launch`  

### NAOqi API

[ALMotion:setAngles](http://doc.aldebaran.com/2-5/naoqi/motion/control-joint-api.html#ALMotionProxy::setAngles__AL::ALValueCR.AL::ALValueCR.floatCR)  

### Sample

```
; angle-ratio: 0.0, arms: :arms
send *ri* :start-grasp

; angle-ratio: 0.3, arms: :rarm
send *ri* :start-grasp 0.3 :rarm
```
