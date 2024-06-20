# ROS node for touchence sensor

## setup touchence sensor
Check your device Switch and LED state.\
Below image is correct state.

![Selection_005](https://user-images.githubusercontent.com/22497720/56508331-e718bc80-655e-11e9-8c22-f3a073406fd1.png)

## build
```
catkin build touchence_serial
source ~/.bashrc
```

## udev setup
You may need `sudo` to copy file to /etc
```
cp `rospack find touchence_serial`/udev/92-touchence.rules /etc/udev/rules.d/
```

## run touchence
```
rosrun touchence_serial touchence_serial_node
```

## check sensor value
```
rostopic echo /touchence/force01
```