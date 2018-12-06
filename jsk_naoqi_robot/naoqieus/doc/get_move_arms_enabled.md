## :get-move-arms-enabled `&optional (arm :arms)` (naoqi_bridge [`kochigami-develop`])

### What is this?

Get the status of whether shaking arms movement is enabled while moving.  

### Parameters

`arm` (optional): `:rarm`, `:larm`, `:arms` (default: `:arms`)   

### Location

`naoqi_apps/launch/locomotion_control.launch`  

### NAOqi API

[ALMotion::getMoveArmsEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::getMoveArmsEnabled__ssCR)  

### Sample

```
; get the status of arms while moving
send *ri* :get-move-arms-enabled
t

; get the status of right arm while moving
send *ri* :get-move-arms-enabled :rarm
nil
```
