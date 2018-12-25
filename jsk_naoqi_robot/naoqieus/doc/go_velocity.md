## :go-velocity `x` `y` `d` `&optional (msec 1000)` `&key (stop t)` (naoqi_driver [`master`])

### What is this?

Move in a specified velocity.  

### Parameters

`x`: velocity along the X axis [m/s] (int, float (-1 ~ 1))  
`y`: velocity along the Y axis [m/s] (int, float (-1 ~ 1))  
`d`: velocity around the Z axis [rad/s] (int, float (-1 ~ 1))  
`msec` (optional): how long a robot keeps moving (int, enable if `:stop t`)  
`stop`(key value): determines whether a robot stops after moving for some time

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALMotion::move](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#almotionproxy-move1)  

### Sample

```
; move vx=0.2 m/s, vy=0.3 m/s and rotate 0.1 rad/s 
send *ri* :go-velocity 0.2 0.3 0.1

; move vx=0.2 m/s, vy=0.3 m/s and rotate 0.1 rad/s, a robot keeps moving forever 
send *ri* :go-velocity 0.2 0.3 0.1 :stop nil

; move vx=0.2 m/s, vy=0.3 m/s and rotate 0.1 rad/s, a robot keeps moving for 2 sec
send *ri* :go-velocity 0.2 0.3 0.1 2000
```