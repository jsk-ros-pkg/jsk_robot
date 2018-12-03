# How to controll NAO/ Pepper with a gazebo simulator + roseus?

## NAO

For environment setup, please refer to [here](https://github.com/ros-naoqi/nao_virtual/tree/master/nao_gazebo_plugin).  
```
roslaunch nao_gazebo_plugin nao_gazebo_plugin_H25.launch
roseus nao-interface.l
(nao-init)
```

## Pepper

For environment setup, please refer to [here](https://github.com/ros-naoqi/pepper_virtual/tree/master/pepper_gazebo_plugin).  
```
roslaunch pepper_gazebo_plugin pepper_gazebo_plugin_Y20.launch
roseus pepper-interface.l
(pepper-init) 
```
## How to move the joint angle of robots?

Same as when we controll real robots.

```
(send *pepper* :head :neck-p :joint-angle 10)
(send *ri* :angle-vector (send *pepper* :angle-vector))
```
[Joints of NAO](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/naoeus/README.md#joints-of-nao)
[Joints of Pepper](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/peppereus/README.md#joints-of-pepper)

## Current issues

[Pepper in falling down & go-velocity](https://github.com/ros-naoqi/pepper_robot/pull/31)  
[go-pos](https://github.com/jsk-ros-pkg/jsk_robot/pull/685)