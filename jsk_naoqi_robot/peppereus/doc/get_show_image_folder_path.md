## :get-show-image-folder-path (naoqi_bridge [`kochigami-develop`])

### What is this?

Get the current path of a file which you want to show on the tablet.  

Related PR is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

### Location

`naoqi_apps/launch/tablet.launch`  

### Sample

```
send *ri* :get-show-image-folder-path
"/home/nao/.local/share/PackageManager/apps/img/html/"
```
