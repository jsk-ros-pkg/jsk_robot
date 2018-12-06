## :error-vector

### What is this?

Return the difference of reference joint angle and sensored joint angle.

Note: This method cannot be used because it requires `naoqi_driver_py/launch/naoqi_driver.launch`. We use `naoqi_driver/launch/naoqi_driver.launch` now.

Related PR is [here](https://github.com/ros-naoqi/naoqi_bridge/pull/37)  

### Sample

```
send *ri* :error-vector
#f(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
```
