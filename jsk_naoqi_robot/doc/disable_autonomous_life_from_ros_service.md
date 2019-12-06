# Disable AutonomousLife from ROS service

## NAO

Please execute this after you [launch jsk_nao_startup.launch](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot/jsk_nao_startup#running-startup-program).

`rosservice call /nao_robot/pose/life/disable`

## Pepper

Please execute this after you [launch jsk_pepper_startup.launch](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot/jsk_pepper_startup#running-startup-program).

`rosservice call /pepper_robot/pose/life/disable`

Or please push a gear mark button of NAOqi dashboard and select `Disable Life`.

