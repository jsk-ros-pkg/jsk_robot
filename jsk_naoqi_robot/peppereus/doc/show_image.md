## :show-image `file` (naoqi_bridge [`kochigami-develop`])

### What is this?

Show an image on the tablet, using the cache. The image should be under `/home/nao/.local/share/PackageManager/apps/<Folder path>/html/` inside a robot.  
(Default is "/home/nao/.local/share/PackageManager/apps/img/html/")  You can change 'Folder path' by using `:set-show-image-folder-path name`.

### Parameters

`file`: file name (str)

### Location

`naoqi_apps/launch/tablet.launch`  

### NAOqi API

[ALTabletService::showImage](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html#ALTabletService::showImage__ssCR)  
Related PR is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

### Sample

```
; put 'test.jpg' under /home/nao/.local/share/PackageManager/apps/img/html/
send *ri* :show-image "test.jpg"

; The ip of the robot from the tablet is 198.18.0.1, and this parent service actually calls ALTabletService::showImage("http://198.18.0.1/img/test.jpg").
```
