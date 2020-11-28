```
roslaunch jsk_hrp2jsknts_startup hrp2jsknts_2dnav.launch
```
```
rosrun jsk_footstep_controller base-controller.l
```
```
rosnode kill pr2_teleop # if real robot. temporary
```
You can send 2d nav goal through rviz.