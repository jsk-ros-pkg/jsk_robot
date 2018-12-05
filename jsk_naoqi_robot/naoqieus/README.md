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

- `:animated-speak str` (naoqi_bridge [kochigami-develop]/ animated_speech.launch)

Speak a sentence and animate it.  

`str`: speech sentence (str)

[ALAnimatedSpeechProxy::say](http://doc.aldebaran.com/2-5/naoqi/audio/alanimatedspeech-api.html#ALAnimatedSpeechProxy::say__ssCR)
[related commit (not a PR to master)](https://github.com/kochigami/naoqi_bridge/tree/add-animated-speak)

```
; Robot speaks the sentence with some gesture.
send *ri* :animated-speak "Hello. Nice to meet you."
```

- `:set-master-volume volume` (naoqi_driver [kochigami-develop]/ naoqi_driver.launch)

Sets the overall output volume of the system.  

`volume`: volume (int 0-100)

[ALAudioDeviceProxy::setOutputVolume](http://doc.aldebaran.com/2-5/naoqi/audio/alaudiodevice-api.html#alaudiodevice-api)  
[related PR](https://github.com/jsk-ros-pkg/jsk_robot/pull/814)

```
send *ri* :set-master-volume 30 ; set master volume as 30 (0~100)
```

- `:get-master-volume` (naoqi_driver [kochigami-develop]/ naoqi_driver.launch)

Gets the overall output volume of the system.  
[ALAudioDeviceProxy::getOutputVolume](http://doc.aldebaran.com/2-5/naoqi/audio/alaudiodevice-api.html#alaudiodevice-api)  
[related PR](https://github.com/jsk-ros-pkg/jsk_robot/pull/814)  

```
2.irteusgl$ send *ri* :get-master-volume
30 ; master volume is set as 30
```

- `:fade-leds led_name r g b a sec` (naoqi_driver [kochigami-develop]/ naoqi_driver.launch)

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
- `:reset-leds led_name` (naoqi_driver [kochigami-develop]/ naoqi_driver.launch)

Set a LED or Group of LEDs to their default state.  

`led_name`: name of the RGB LED or Group. (string, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds.html#groups-short-names-and-names).)  

[ALLedsProxy::reset](http://doc.aldebaran.com/2-5/naoqi/sensors/alleds-api.html#alleds-api)  
[related PR](https://github.com/jsk-ros-pkg/jsk_robot/pull/999)

```
send *ri* :reset-leds "FaceLeds" ;; Pepper's eye becomes clear
```

- `:servo-on` (naoqi_bridge/naoqi_pose [master])

A robot sets motor on takes an initial pose.  

[ALMotionProxy::wakeUp](http://doc.aldebaran.com/2-5/naoqi/motion/control-stiffness-api.html#ALMotionProxy::wakeUp)  

```
send *ri* :servo-on
```

- `:servo-off` (naoqi_bridge/naoqi_pose [master])

A robot sets motor off and takes a reset pose.  

[ALMotionProxy::rest](http://doc.aldebaran.com/2-5/naoqi/motion/control-stiffness-api.html#ALMotionProxy::rest)

```
send *ri* :servo-off
```

- `:speak str` (naoqi_driver [master])

Speak a sentence.  

`str`: speech sentence (str)

[ALTextToSpeechProxy::say](http://doc.aldebaran.com/2-5/naoqi/audio/altexttospeech-api.html#ALTextToSpeechProxy::say__ssCR)  

```
send *ri* :speak "Hello. Nice to meet you."
```

- `:start-grasp angle-ratio arm` (naoqi_bridge/naoqi_pose [master])

Start grasping.  

`angle-ratio`: ratio of grasping (float, 0.5-1.0 (default 0.0))  
`arm`: grasping arm type (str, `:larm`, `:rarm`, `:arms` (default `:arms`))  

[ALMotion:setAngles](http://doc.aldebaran.com/2-5/naoqi/motion/control-joint-api.html#ALMotionProxy::setAngles__AL::ALValueCR.AL::ALValueCR.floatCR)  

```
; angle-ratio: 0.0, arms: :arms
send *ri* :start-grasp

; angle-ratio: 0.3, arms: :rarm
send *ri* :start-grasp 0.3 :rarm
```

- `:stop-grasp angle-ratio arm` (naoqi_bridge/naoqi_pose [master])

Stop grasping.  

`angle-ratio`: ratio of grasping (float, 0.5-1.0 (default 1.0))  
`arm`: grasping arm type (str, `:larm`, `:rarm`, `:arms` (default `:arms`))  

[ALMotion:setAngles](http://doc.aldebaran.com/2-5/naoqi/motion/control-joint-api.html#ALMotionProxy::setAngles__AL::ALValueCR.AL::ALValueCR.floatCR)  

```
; angle-ratio: 1.0, arms: :arms
send *ri* :stop-grasp

; angle-ratio: 0.6, arms: :larm
send *ri* :stop-grasp 0.6 :larm
```

- `:enable-life` (naoqi_bridge/naoqi_pose [master])

Enable AutonomousLife.  

[ALAutonomousLife::setState](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomouslife-api.html#ALAutonomousLifeProxy::setState__ssCR)  

```
send *ri* :enable-life
```

- `:disable-life` (naoqi_bridge/naoqi_pose [master])

Disable AutonomousLife.  

[ALAutonomousLife::setState](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomouslife-api.html#ALAutonomousLifeProxy::setState__ssCR)  

```
send *ri* :disable-life
```

- `:get-life` (naoqi_bridge/naoqi_pose [master])

Return AutonomousLife state. ("solitary", "interactive", "safeguard", "disabled")    

[ALAutonomousLife::getState](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomouslife-api.html#ALAutonomousLifeProxy::getState)  
For further details of AutonomousLife state, please refer to [here](http://doc.aldebaran.com/2-5/ref/life/state_machine_management.html#autonomouslife-states).

```
send *ri* :get-life
"disabled"
```

- `:set-language language` (naoqi_driver [master])

Set a language which a robot speaks.  

`language`: language (str)  

[ALTextToSpeech::setLanguage](http://doc.aldebaran.com/2-5/naoqi/audio/altexttospeech-api.html#ALTextToSpeechProxy::setLanguage__ssCR)  

```
send *ri* :set-language "Japanese" ; "English"
```

- `:get-language` (naoqi_driver [master])

Get a language which a robot speaks.  

[ALTextToSpeech::getLanguage](http://doc.aldebaran.com/2-5/naoqi/audio/altexttospeech-api.html#ALTextToSpeechProxy::getLanguage)  

```
send *ri* :get-language
"Japanese"
```

- `:go-pos x y theta` (naoqi_driver [master])

Move to a specified distance.  

`x`: distance along the X axis [m] (int, float)  
`y`: distance along the Y axis [m] (int, float)  
`theta`: rotation around the Z axis [degree] (int)

[ALMotion::moveTo](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#almotionproxy-moveto1)  

```
; move to x=1.0 y=2.0 and rotate 30 degree 
send *ri* :go-pos 1.0 2.0 30
```

- `:go-velocity x y d &optional (msec 1000) &key (stop t)` (naoqi_driver [master])

Move in a specified velocity.  

`x`: velocity along the X axis [m/s] (int, float (-1~1))  
`y`: velocity along the Y axis [m/s] (int, float (-1~1))  
`theta`: velocity around the Z axis [rad/s] (int, float (-1~1))  
`msec` (optional): how long a robot keeps moving (int, enable if `:stop t`)  
`stop`(key value): determines whether a robot stops after moving for some time  

[ALMotion::move](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#almotionproxy-move1)  

```
; move vx=0.2 m/s, vy=0.3 m/s and rotate 0.1 rad/s 
send *ri* :go-velocity 0.2 0.3 0.1

; move vx=0.2 m/s, vy=0.3 m/s and rotate 0.1 rad/s, a robot keeps moving forever 
send *ri* :go-velocity 0.2 0.3 0.1 :stop nil

; move vx=0.2 m/s, vy=0.3 m/s and rotate 0.1 rad/s, a robot keeps moving for 2 sec
send *ri* :go-velocity 0.2 0.3 0.1 2000
```

- `:play-audio-file file` (naoqi_driver [kochigami-develop])

Play audio file inside a robot.  

`file`: path to your audio file (str)  
Please locate mp3, wav file under `/home/nao/`. Then, set a path to your audio file to `file`.  

[ALAudioPlayer::playFile](http://doc.aldebaran.com/2-5/naoqi/audio/alaudioplayer-api.html#ALAudioPlayerProxy::playFile__ssCR)  

This is an example of how to play test.mp3 file from `/home/nao/audio_file/test.mp3` path.  

```
; ssh nao@<robot IP> 
; then, create audio_file folder and put test.mp3 file
; scp test.mp3  nao@<robot IP>:/home/nao/audio_file/

send *ri* :play-audio-file "/audio_file/test.mp3"
```

- `:set-external-collision-protection-status type status` (naoqi_bridge/naoqi_apps [kochigami-develop])

Enable/disable external collision protection of a robot on the given name.  

`type`: body parts type of a robot (int)  

```
All:  0
Move: 1
Arms: 2
Larm: 3
Rarm: 4 
```

`status`: t/ nil (bool)  

[ALMotion::setExternalCollisionProtectionEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/reflexes-external-collision-api.html#ALMotionProxy::setExternalCollisionProtectionEnabled__ssCR.bCR)  

```
; disable external collision protection for Move part
send *ri* :set-external-collision-protection-status 2 nil
```

- `:get-external-collision-protection-status type` (naoqi_bridge/naoqi_apps [kochigami-develop])

Check if the external collision protection is activated on the given name.  

`type`: body parts type of a robot (int)  

```
All:  0
Move: 1
Arms: 2
Larm: 3
Rarm: 4 
```

[ALMotion::getExternalCollisionProtectionEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/reflexes-external-collision-api.html#ALMotionProxy::getExternalCollisionProtectionEnabled__ssCR)  

```
; get status of external collision protection for Move part
send *ri* :get-external-collision-protection-status 2 
t
```

- `:set-background-movement-enabled status` (naoqi_bridge/naoqi_apps [kochigami-develop])

Enable or disable the background movements. For further details on background movement, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albackgroundmovement.html#albackgroundmovement).   

`status`: t/ nil (bool)  

[ALBackgroundMovement::setEnabled](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albackgroundmovement-api.html#ALBackgroundMovementProxy::setEnabled__b)  

```
; enable background movement
send *ri* :set-background-movement-enabled t
```

- `:get-background-movement-enabled` (naoqi_bridge/naoqi_apps [kochigami-develop])

Return whether the background movements are enabled. For further details on background movement, please refer to [here](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albackgroundmovement.html#albackgroundmovement).   

[ALBackgroundMovement::isEnabled](http://doc.aldebaran.com/2-5/naoqi/interaction/autonomousabilities/albackgroundmovement-api.html#ALBackgroundMovementProxy::isEnabled)  

```
; get background movement status
send *ri* :get-background-movement-enabled
t
```

- `:set-move-arms-enabled status &optional (arm :arms)` (naoqi_bridge/naoqi_apps [kochigami-develop])

Enable shaking arms movement while moving

`status`: t/ nil (bool)  
`arm` (optional): `:rarm`, `:larm`, `:arms` (default: `:arms`)   

[ALMotion::setMoveArmsEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::setMoveArmsEnabled__bCR.bCR)  

```
; move arms while moving
send *ri* :set-move-arms-enabled t

; do not move right arm while moving
send *ri* :set-move-arms-enabled nil :rarm
```

- `:get-move-arms-enabled &optional (arm :arms)` (naoqi_bridge/naoqi_apps [kochigami-develop])

Get the status of whether shaking arms movement is enabled while moving.  

`arm` (optional): `:rarm`, `:larm`, `:arms` (default: `:arms`)   

[ALMotion::getMoveArmsEnabled](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::getMoveArmsEnabled__ssCR)  

```
; get the status of arms while moving
send *ri* :get-move-arms-enabled
t

; get the status of right arm while moving
send *ri* :get-move-arms-enabled :rarm
nil
```

- `:error-vector`

Return the difference of reference joint angle and sensored joint angle.

Note: This method cannot be used because it requires `naoqi_driver_py/launch/naoqi_driver.launch`. We use `naoqi_driver/launch/naoqi_driver.launch` now.  
[related PR](https://github.com/ros-naoqi/naoqi_bridge/pull/37)  

```
send *ri* :error-vector
#f(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
```

