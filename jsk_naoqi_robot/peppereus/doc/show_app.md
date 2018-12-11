## :show-app `app` (naoqi_bridge [`kochigami-develop`])

### What is this?

Start new application on tablet and shows it. The index.html file of the app should be in `/home/nao/.local/share/PackageManager/apps/<app>/html/`. 'app' is a parameter of this method.

### Parameters

`app`: app name (str)

### Location

`naoqi_apps/launch/tablet.launch`  

### NAOqi API

[ALTabletService::loadApplication](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html#ALTabletService::loadApplication__ssCR)  
[ALTabletService::showWebview](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html#altabletservice-showwebview1)  
Related PR is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

### Sample

```
; create app named 'img' under /home/nao/.local/share/PackageManager/apps/img/html/
send *ri* :show-app "img"
```
