## :show-webview `url` (naoqi_bridge [`kochigami-develop`])

### What is this?

Display the webview on the tablet and load the url.

### Parameters

`url`: url of the web page (str)

### Location

`naoqi_apps/launch/tablet.launch`  

### NAOqi API

[ALTabletService::showWebview](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html#ALTabletService::showWebview__ssCR)  
Related PR is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

### Sample

```
send *ri* :show-webview "http://www.jsk.t.u-tokyo.ac.jp/index-j.html"
```
