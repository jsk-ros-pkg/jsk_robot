naoqieus
=========

This is a common package for naoeus and peppereus.
This is used when controlling NAO and Pepper via roseus. 
Basic methods for NAO and Pepper are stored.

How to try methods
------------------

1. [roslaunch jsk_pepper_startup.launch](../jsk_pepper_statup/README.md) or [roslaunch jsk_nao_startup.launch](../jsk_nao_statup/README.md)  
2. [launch peppereus](../peppereus/README.md) or [launch naoeus](../naoeus/README.md). Please refer to `Control NAO/ Pepper via roseus`.  
3. Please try methods, you can refer to the explanations below how to try them. If there is a sign of `kochigami-develop`, please follow [Interface when controlling NAO and Pepper via roseus](../README.md).  

Methods
-------

- [:animated-speak `str` (naoqi_bridge [`kochigami-develop`])](doc/animated_speak.md)  

- [:disable-life (naoqi_bridge [`master`])](doc/disable_life.md)  

- [:enable-life (naoqi_bridge [`master`])](doc/enable_life.md)  

- [:error-vector](doc/error_vector.md)  

- [:fade-leds `led_name` `r` `g` `b` `sec` (naoqi_driver [`kochigami-develop`])](doc/fade_leds.md)  

- [:get-background-movement-enabled (naoqi_bridge [`kochigami-develop`])](doc/get_background_movement_enabled.md)  

- [:get-external-collision-protection-status `type` (naoqi_bridge [`kochigami-develop`])](doc/get_external_collision_protection_status.md)  

- [:get-language (naoqi_driver [`master`])](doc/get_language.md)  

- [:get-life (naoqi_bridge [`master`])](doc/get_life.md)  

- [:get-master-volume (naoqi_driver [`kochigami-develop`])](doc/get_master_volume.md)  

- [:get-move-arms-enabled `&optional (arm :arms)` (naoqi_bridge [`kochigami-develop`])](doc/get_move_arms_enabled.md)    

- [:go-pos `x` `y` `theta` (naoqi_driver [`master`])](doc/go_pos.md)  

- [:go-velocity `x` `y` `d` `&optional (msec 1000)` `&key (stop t)` (naoqi_driver [`master`])](doc/go_velocity.md)  

- [:play-audio-file `file` (naoqi_driver [`kochigami-develop`])](doc/play_audio_file.md)  

- [:reset-leds `led_name` (naoqi_driver [`kochigami-develop`])](doc/reset_leds.md)  

- [:servo-on (naoqi_bridge [`master`])](doc/servo_on.md)  

- [:servo-off (naoqi_bridge [`master`])](doc/servo_off.md)  

- [:set-background-movement-enabled `status` (naoqi_bridge [`kochigami-develop`])](doc/set_background_movement_enabled.md)  

- [:set-external-collision-protection-status `type` `status` (naoqi_bridge [`kochigami-develop`])](doc/set_external_collision_protection_status.md)  

- [:set-language `language` (naoqi_driver [`master`])](doc/set_language.md)  

- [:set-master-volume `volume` (naoqi_driver [`kochigami-develop`])](doc/set_master_volume.md)  

- [:set-move-arms-enabled `status` `&optional (arm :arms)` (naoqi_bridge [`kochigami-develop`])](doc/set_move_arms_enabled.md)  

- [:speak `str` (naoqi_driver [`master`])](doc/speak.md)  

- [:start-grasp `&optional (angle-ratio 0.0) (arm :arms)` (naoqi_bridge [`master`])](doc/start_grasp.md)  

- [:stop-grasp `&optional (angle-ratio 1.0) (arm :arms)` (naoqi_bridge [`master`])](doc/stop_grasp.md)  