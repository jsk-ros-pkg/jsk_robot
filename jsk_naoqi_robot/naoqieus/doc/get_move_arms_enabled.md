## :get-move-arms-enabled `&optional (arm :arms)` (naoqi_bridge [`kochigami-develop`])

### What is this?

Get the status of whether shaking arms movement is enabled while moving.  

### Parameters

`arm` (optional): `:rarm`, `:larm`, `:arms` (default: `:arms`)   

### Location

`naoqi_apps/launch/locomotion_control.launch`  

### NAOqi API

[ALMotion::getMoveArmsEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::getMoveArmsEnabled__ssCR)  

Related commit is [here](https://github.com/kochigami/naoqi_bridge/commit/5d36e3d1a7e13095d62831ca4568f44a43f7bc37#diff-e9b6c21fdccdb01cff79b583fc7ad7d2)

### Sample

```
; get the status of arms while moving
send *ri* :get-move-arms-enabled
t

; get the status of right arm while moving
send *ri* :get-move-arms-enabled :rarm
nil
```
