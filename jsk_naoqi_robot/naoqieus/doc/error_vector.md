## :error-vector (naoqi_driver [`kochigami-develop`])

### What is this?

Return the difference of reference joint angle and sensored joint angle.

Related PR is [here](https://github.com/kochigami/naoqi_driver/pull/6)  

Related original PR (for naoqi_driver_py) is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/37)  

### Location

`naoqi_driver/launch/naoqi_driver.launch`  
`naoqi_driver/src/converters/joint_states.cpp`  	   

### Sample

```
send *ri* :error-vector
#f(3.201651e-07 -0.087846 9.562265e-05 -2.561321e-06 2.049057e-05 -1.707547e-06 0.527356 -0.755197 -0.439454 0.816019 1.16013 0.0 0.0 0.0 0.027345 -0.351577 -0.755204 0.0 0.0 0.0)
```
