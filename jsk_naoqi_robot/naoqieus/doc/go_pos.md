## :go-pos `x` `y` `theta` (naoqi_driver [`master`])

### What is this?

Move to a specified distance.  

### Parameters

`x`: distance along the X axis [m] (int, float)  
`y`: distance along the Y axis [m] (int, float)  
`theta`: rotation around the Z axis [degree] (int)

### Location

`launch/naoqi_driver.launch`  

### NAOqi API

[ALMotion::moveTo](http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#almotionproxy-moveto1)  

### Sample

```
; move to x=1.0 y=2.0 and rotate 30 degree 
send *ri* :go-pos 1.0 2.0 30
```
