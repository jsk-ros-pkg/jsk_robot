naoqieus
=========

This is a common package for naoeus and peppereus.
This is used when controlling NAO and Pepper via roseus. 
Basic methods for NAO and Pepper are stored.

How to try methods
------------------

1. [roslaunch jsk_pepper_startup.launch](../jsk_pepper_statup/README.md) or [roslaunch jsk_nao_startup.launch](../jsk_nao_statup/README.md)
2. [launch peppereus](../peppereus/README.md) or [launch naoeus](../naoeus/README.md). Please refer to `Control NAO/ Pepper via roseus`.
3. Please try methods, you can refer to the explanations below how to try them. If there is a sign of `kochigami-develop`, please follow [Interface when controlling NAO and Pepper via roseus](../README.md).  

Methods
-------

- `:set-master-volume volume` (kochigami-develop)

Sets the overall output volume of the system.  

`volume`: volume (int 0-100)

[ALAudioDeviceProxy::setOutputVolume](http://doc.aldebaran.com/2-5/naoqi/audio/alaudiodevice-api.html#alaudiodevice-api)  
[related PR](https://github.com/jsk-ros-pkg/jsk_robot/pull/814)

```
send *ri* :set-master-volume 30 ; set master volume as 30 (0~100)
```

- `:get-master-volume` (kochigami-develop)

Gets the overall output volume of the system.  
[ALAudioDeviceProxy::getOutputVolume](http://doc.aldebaran.com/2-5/naoqi/audio/alaudiodevice-api.html#alaudiodevice-api)  
[related PR](https://github.com/jsk-ros-pkg/jsk_robot/pull/814)  

```
2.irteusgl$ send *ri* :get-master-volume
30 ; master volume is set as 30
```

- `:fade-leds led_name r g b a sec` (kochigami-develop)

Sets the color of an RGB led using RGB color code.  

`led_name`: name of the RGB LED or Group. (string, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds.html#groups-short-names-and-names).)  
`r`: intensity of red channel (float 0-1)  
`g`: intensity of green channel (float 0-1)  
`b`: intensity of blue channel (float 0-1)  
`d`: time used to fade in seconds (int)  

[ALLedsProxy::fadeRGB with RGB color code](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds-api.html#alleds-api)  
[related PR](https://github.com/jsk-ros-pkg/jsk_robot/pull/999)

```
send *ri* :fade-leds "FaceLeds" 0.5 0.5 0 0 1 ;; Robot's eyes become yellow in a 1 sec.
```
- `:reset-leds led_name` (kochigami-develop)

Set a LED or Group of LEDs to their default state.  

`led_name`: name of the RGB LED or Group. (string, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds.html#groups-short-names-and-names).)  

[ALLedsProxy::reset](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds-api.html#alleds-api)  
[related PR](https://github.com/jsk-ros-pkg/jsk_robot/pull/999)

```
send *ri* :reset-leds "FaceLeds" ;; Pepper's eye becomes clear
```