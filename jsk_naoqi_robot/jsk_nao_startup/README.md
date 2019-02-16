jsk_nao_startup
==================

What's this?
------------
contains JSK's launch file for startup NAO with ROS

Running startup program
-----------------------

```
rossetip
roslaunch jsk_nao_startup jsk_nao_startup.launch network_interface:=<your network interaface (ex. eth0, enp0s31f6...)>
```

% For network_interface variable, please check `ifconfig` for the interface name your PC uses. 