## :hide-image (naoqi_bridge [`kochigami-develop`])

### What is this?

Hide image currently displayed. This method deletes every image, app, webview and shows Pepper's bubbles.  

### Location

`naoqi_apps/launch/tablet.launch`  

### NAOqi API

[ALTabletService::hideImage](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html?highlight=altablet#ALTabletService::hideImage)  
Related PR is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

### Sample

```
send *ri* :hide-image
```
