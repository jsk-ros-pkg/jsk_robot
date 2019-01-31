## :set-basic-awareness-enabled `status` (naoqi_bridge [`kochigami-develop`])

### What is this?

Enable or disable basic awareness. For further details on basic awareness, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albasicawareness.html#albasicawareness).   

### Parameters

`status`: t/ nil (bool)  

### Location

`naoqi_apps/launch/basic_awareness.launch`  

### NAOqi API

[ALBasicAwareness::setEnabled](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albasicawareness-api.html#ALBasicAwarenessProxy::setEnabled__b)  

Related commits are [here](https://github.com/kochigami/naoqi_bridge/commit/f91bb1ed5598d19d5be4f6186b0710c5b69a5a3d#diff-a526e4a93ddc5f0149214c25d7f13988) and [here](https://github.com/kochigami/naoqi_bridge/commit/8344b35a79e3e465cd43ffc1457254f3b13c6ef1#diff-a526e4a93ddc5f0149214c25d7f13988).  

### Sample

```
; enable basic awareness
send *ri* :set-basic-awareness-enabled t
```