# naoqieus

This is a common package for naoeus and peppereus.
This is used when controlling NAO and Pepper via roseus. 
Basic methods for NAO and Pepper are stored.

## How to try methods

1. [roslaunch jsk_pepper_startup.launch](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot/jsk_pepper_startup#running-startup-program) or [roslaunch jsk_nao_startup.launch](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot/jsk_nao_startup#running-startup-program)  
2. [launch peppereus](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot/peppereus#control-pepper-via-roseus) or [launch naoeus](https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_naoqi_robot/naoeus/README.md#control-nao-via-roseus).   
3. Please try methods, you can refer to the explanations below how to try them. If there is a sign of `kochigami-develop`, please follow [Interface when controlling NAO and Pepper via roseus](https://github.com/jsk-ros-pkg/jsk_robot/tree/master/jsk_naoqi_robot#interface-when-controlling-nao-and-pepper-via-roseus).  

## Methods

- [:animated-speak `str` (naoqi_bridge [`kochigami-develop`])](doc/animated_speak.md)  

- [:disable-life (naoqi_bridge [`master`])](doc/disable_life.md)  

- [:enable-life (naoqi_bridge [`master`])](doc/enable_life.md)  

- [:error-vector](doc/error_vector.md)  

- [:fade-leds `led_name` `r` `g` `b` `sec` (naoqi_driver [`kochigami-develop`])](doc/fade_leds.md)  

- [:get-background-movement-enabled (naoqi_bridge [`kochigami-develop`])](doc/get_background_movement_enabled.md)

- [:get-basic-awareness-enabled (naoqi_bridge [`kochigami-develop`])](doc/get_basic_awareness_enabled.md)  

- [:get-defined-pose-list (naoqi_pose [`master`])](doc/get_defined_pose_list.md)  

- [:get-external-collision-protection-status `type` (naoqi_bridge [`kochigami-develop`])](doc/get_external_collision_protection_status.md)  

- [:get-language (naoqi_driver [`master`])](doc/get_language.md)  

- [:get-life (naoqi_bridge [`master`])](doc/get_life.md)  

- [:get-master-volume (naoqi_driver [`kochigami-develop`])](doc/get_master_volume.md)  

- [:get-move-arms-enabled `&optional (arm :arms)` (naoqi_bridge [`kochigami-develop`])](doc/get_move_arms_enabled.md)    

- [:get-take-picture-folder-path (naoqi_bridge [`kochigami-develop`])](doc/get_take_picture_folder_path.md)

- [:go-pos `x` `y` `theta` (naoqi_driver [`master`])](doc/go_pos.md)  

- [:go-velocity `x` `y` `d` `&optional (msec 1000)` `&key (stop t)` (naoqi_driver [`master`])](doc/go_velocity.md)  

- [:play-audio-file `file` (naoqi_driver [`kochigami-develop`])](doc/play_audio_file.md)  

- [:reset-leds `led_name` (naoqi_driver [`kochigami-develop`])](doc/reset_leds.md)  

- [:servo-on (naoqi_bridge [`master`])](doc/servo_on.md)  

- [:servo-off (naoqi_bridge [`master`])](doc/servo_off.md)  

- [:set-background-movement-enabled `status` (naoqi_bridge [`kochigami-develop`])](doc/set_background_movement_enabled.md)

- [:set-basic-awareness-enabled `status` (naoqi_bridge [`kochigami-develop`])](doc/set_basic_awareness_enabled.md)  

- [:set-body-pose-with-speed `posture-name` `&optional (speed 0.7)` (naoqi_pose [`master`])](doc/set_body_pose_with_speed.md)  

- [:set-external-collision-protection-status `type` `status` (naoqi_bridge [`kochigami-develop`])](doc/set_external_collision_protection_status.md)  

- [:set-language `language` (naoqi_driver [`master`])](doc/set_language.md)  

- [:set-master-volume `volume` (naoqi_driver [`kochigami-develop`])](doc/set_master_volume.md)  

- [:set-move-arms-enabled `status` `&optional (arm :arms)` (naoqi_bridge [`kochigami-develop`])](doc/set_move_arms_enabled.md)  

- [:set-take-picture-folder-path `name`(naoqi_bridge [`kochigami-develop`])](doc/set_take_picture_folder_path.md) 

- [:speak `str` (naoqi_driver [`master`])](doc/speak.md)  

- [:speak-action `str` `&optional (wait 60)` (naoqi_driver [`master`] and naoqi_apps [`kochigami-develop`])](doc/speak_action.md)  

- [:start-grasp `&optional (angle-ratio 0.0) (arm :arms)` (naoqi_bridge [`master`])](doc/start_grasp.md)  

- [:stop-grasp `&optional (angle-ratio 1.0) (arm :arms)` (naoqi_bridge [`master`])](doc/stop_grasp.md)

- [:take-picture `file-name` (naoqi_bridge [`kochigami-develop`])](doc/take_picture.md)