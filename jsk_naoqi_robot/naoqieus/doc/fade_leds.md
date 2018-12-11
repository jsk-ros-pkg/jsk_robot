## :fade-leds `led_name` `r` `g` `b` `sec` (naoqi_driver [`kochigami-develop`])

### What is this?

Sets the color of an RGB led using RGB color code.  

### Parameters

`led_name`: name of the RGB LED or Group. (string, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds.html#groups-short-names-and-names).)  
`r`: intensity of red channel (float 0-1)  
`g`: intensity of green channel (float 0-1)  
`b`: intensity of blue channel (float 0-1)  
`d`: time used to fade in seconds (int/ float)  

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALLeds::fadeRGB with RGB color code](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds-api.html#alleds-api)  

Related PR is [here](https://github.com/ros-naoqi/naoqi_driver/pull/100) and [here](https://github.com/jsk-ros-pkg/jsk_robot/pull/999)

### Sample

```
send *ri* :fade-leds "FaceLeds" 0.5 0.5 0 1 ;; Robot's eyes become yellow in a 1 sec.
```