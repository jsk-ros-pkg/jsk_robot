## :reset-leds `led_name` (naoqi_driver [`kochigami-develop`])

### What is this?

Set a LED or Group of LEDs to their default state.  

### Parameters

`led_name`: name of the RGB LED or Group. (string, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds.html#groups-short-names-and-names).)  

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALLedsProxy::reset](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds-api.html#alleds-api)  

Related PR is [here](https://github.com/ros-naoqi/naoqi_driver/pull/100) and [here](https://github.com/jsk-ros-pkg/jsk_robot/pull/999)

### Sample

```
send *ri* :reset-leds "FaceLeds" ;; Pepper's eye becomes clear
```