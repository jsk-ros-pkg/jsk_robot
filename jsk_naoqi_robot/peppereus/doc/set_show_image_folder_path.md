## :set-show-image-folder-path `name` (naoqi_bridge [`kochigami-develop`])

### What is this?

Change the path of a file which you want to show on the tablet. This method changes the part of `Folder path` in `/home/nao/.local/share/PackageManager/apps/<Folder path>/html/`.  

Related PR is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

### Parameters

`name`: directory name (str, default is `img`)  

### Location

`naoqi_apps/launch/tablet.launch`  


### Sample

```
send *ri* :set-show-image-folder-path "aaa/bbb"
#<naoqi_bridge_msgs::setfolderpathresponse #X9b1c660>

; put 'test.jpg' under /home/nao/.local/share/PackageManager/apps/aaa/bbb/html/
; (send *ri* :get-show-image-folder-path) will return "/home/nao/.local/share/PackageManager/apps/aaa/bbb/html/"

send *ri* :show-image "test.jpg"
```
