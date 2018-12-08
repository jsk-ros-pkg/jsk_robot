## :get-master-volume (naoqi_driver [`kochigami-develop`])

### What is this?

Gets the overall output volume of the system.  

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALAudioDevice::getOutputVolume](http://doc.aldebaran.com/2-5/naoqi/audio/alaudiodevice-api.html#alaudiodevice-api)

Related PR is [here](https://github.com/jsk-ros-pkg/jsk_robot/pull/814)  

### Sample

```
2.irteusgl$ send *ri* :get-master-volume
30 ; master volume is set as 30
```