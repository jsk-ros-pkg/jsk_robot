## :take-picture `file-name` (naoqi_bridge [`kochigami-develop`])

### What is this?

Takes one picture and store it in PC of Naoqi robot.

### Parameters

`file-name`: file name (str)

### Location

`naoqi_apps/launch/photo_capture.launch`

### NAOqi API

[ALPhotoCaptureProxy::takePicture](http://doc.aldebaran.com/2-5/naoqi/vision/alphotocapture-api.html#ALPhotoCaptureProxy::takePicture__ssCR.ssCR)

Related commit is [here](https://github.com/kochigami/naoqi_bridge/pull/8)

### Sample

```
; nao.jpg is stored in `/home/nao/.local/share/PackageManager/apps/img/html/test.jpg`

send *ri* :take-picture "nao"
```