## :set-move-arms-enabled `status` `&optional (arm :arms)` (naoqi_bridge [`kochigami-develop`])

### What is this?

Enable shaking arms movement while moving

### Parameters

`status`: t/ nil (bool)  
`arm` (optional): `:rarm`, `:larm`, `:arms` (default: `:arms`)   

### Location

`naoqi_apps/launch/locomotion_control.launch`  

### NAOqi API

[ALMotion::setMoveArmsEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::setMoveArmsEnabled__bCR.bCR)  

### Sample

```
; move arms while moving
send *ri* :set-move-arms-enabled t

; do not move right arm while moving
send *ri* :set-move-arms-enabled nil :rarm
```
