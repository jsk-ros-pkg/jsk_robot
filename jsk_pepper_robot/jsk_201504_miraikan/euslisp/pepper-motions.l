#!/usr/bin/env roseus

;; reset pose                                                                  
(setq *reset_pose* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))

;; *touch_ears*
;; used when introducing speaker
(setq *touch_ears* #f(20.0 -2.0 -30.0 -15.0 20.0 -70.0 -88.0 -40.0 -15.0 -20.0 70.0 88.0 40.0 0.0 30.0))

;; *lift_right_arm* 
;; lift right arm up to head
(setq *left_right_arm* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 -30.0 -20.0 50.0 50.0 -50.0 0.0 0.0))

;; *bow*
(setq *bow* #f(2.0 -2.0 -40.0 85.0 10.0 -70.0 -20.0 -40.0 50.0 -1.0 -1.0 50.0 50.0 0.0 20.0))

;; *lift_arm_and_bow*
;; first lift arm, then bow. used at fist
(setq *lift_arm_and_bow* (list *left_right_arm* *left_right_arm* *left_right_arm* *bow* *bow* *bow* *reset_pose*)) 

;; *touch_left_head*
(setq *touch_lift_head* #f(2.0 10.0 -5.0 -60.0 60.0 -30.0 -89.5 -60.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))

;; *touch_CPU*
(setq *touch_CPU* #f(28.0 0.0 -57.0 -80.0 30.0 -30.0 -89.5 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0))

;; *look_foot*
;; used when introducing touch sensor on foot
(setq *look_foot* #f(2.0 -20.0 -20.0 85.0 10.0 -70.0 -20.0 -40.0 50.0 -10.0 70.0 20.0 -70.0 -20.0 20.0))
(setq *open_pepper* #f(2.0 -2.0 -5.0 10.0 80.0 -70.0 -40.0 -40.0 10.0 -80.0 70.0 40.0 40.0 0.0 0.0))

;; *speaker_touch_sensor* 
;; speaker and touchsensor 
(setq *speaker_touch_sensor* (list *touch_ears* *touch_ears* *touch_ears* *touch_ears* *open_pepper* *open_pepper* *look_foot* *look_foot* *look_foot* *reset_pose*))

;; *look_right* *look_left*
;; used when look around

;; *look_right*
(setq *look_right* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 -20.0 -10.0))
;; *look_left:
(setq *look_left* #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 20.0 -10.0))

;; *show the camera on head*
(setq *show_the_camera_on_head* #f(2.0 -2.0 -5.0 10.0 10.0 -60.0 -89.5 -40.0 10.0 -10.0 60.0 89.5 40.0 0.0 5.0))

;; *show the camera in mouth*
(setq *show_the_camera_in_mouth* #f(2.0 -2.0 -5.0 10.0 10.0 -60.0 -89.5 -40.0 10.0 -10.0 60.0 89.5 40.0 0.0 -35.0))

;;open two hands widely
(setq *open_two_hands_widely* #f(0.0 0.0 0.0 60.0 0.0 -40.0 -30.0 -104.5 -70.0 0.0 -50.0 40.0 104.5 0.0 0.0))

;; *show_the_camera*
(setq *show_the_camera* (list *reset_pose* *reset_pose* *reset_pose* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_on_head* *show_the_camera_in_mouth* *open_two_hands_widely* *reset_pose*))

;;*pepper_cannot*
;; used when introducing what pepper can't do
(setq *pepper_cannot1* #f(2.0 -2.0 -5.0 100.0 10.0 -70.0 -70.0 -50.0 40.0 -10.0 70.0 70.0 50.0 0.0 0.0))
(setq *pepper_cannot2* #f(2.0 -2.0 -5.0 100.0 10.0 -70.0 -70.0 -50.0 40.0 -10.0 70.0 70.0 50.0 40.0 0.0))
(setq *pepper_cannot3* #f(2.0 -2.0 -5.0 100.0 10.0 -70.0 -70.0 -50.0 40.0 -10.0 70.0 70.0 50.0 -40.0 0.0))
(setq *pepper_cannot4* #f(2.0 -2.0 -5.0 100.0 10.0 -70.0 -70.0 50.0 40.0 -10.0 70.0 70.0 -50.0 0.0 0.0))
(setq *pepper_cannot5* #f(2.0 -2.0 -5.0 100.0 10.0 -70.0 -70.0 50.0 40.0 -10.0 70.0 70.0 -50.0 40.0 0.0))
(setq *pepper_cannot6* #f(2.0 -2.0 -5.0 100.0 10.0 -70.0 -70.0 50.0 40.0 -10.0 70.0 70.0 -50.0 -40.0 0.0))

;;*pepper_can*
;; used when introducing what pepper can do
(setq *pepper_open_hand* #f(2.0 -2.0 -5.0 30.0 30.0 -70.0 -20.0 -40.0 30.0 -30.0 70.0 20.0 40.0 0.0 0.0))
(setq *pepper_run1* #f(2.0 -10.0 -15.0 110.0 10.0 -100.0 -80.0 -40.0 60.0 -10.0 70.0 70.0 40.0 0.0 0.0))
(setq *pepper_run2* #f(2.0 10.0 -15.0 60.0 10.0 -70.0 -70.0 40.0 110.0 -10.0 100.0 80.0 -40.0 0.0 0.0))
;;move hip front
(setq *pepper_can1* #f(2.0 -2.0 -5.0 50.0 30.0 -70.0 -80.0 -90.0 50.0 -30.0 70.0 80.0 90.0 0.0 0.0)) 
;;move hip right
(setq *pepper_can2* #f(2.0 -20.0 -5.0 50.0 30.0 -70.0 -80.0 -90.0 50.0 -30.0 70.0 80.0 90.0 -20.0 0.0))
;;move hip left 
(setq *pepper_can3* #f(2.0 20.0 -5.0 50.0 30.0 -70.0 -80.0 -90.0 50.0 -30.0 70.0 80.0 90.0 20.0 0.0)) 
(setq *pepper_can4* #f(2.0 0.0 -5.0 20.0 30.0 -70.0 -20.0 -90.0 20.0 -30.0 70.0 20.0 90.0 0.0 0.0))
(setq *pepper_can5* #f(2.0 0.0 -5.0 20.0 30.0 -70.0 -20.0 -90.0 20.0 -30.0 70.0 20.0 90.0 0.0 30.0))
(setq *pepper_can6* #f(2.0 0.0 -5.0 20.0 30.0 -70.0 -20.0 -90.0 20.0 -30.0 70.0 20.0 90.0 0.0 -30.0))

;; *pepper_can_cannot*
(setq *pepper_can_cannot* (list *pepper_cannot4* *pepper_cannot5* *pepper_cannot4* *pepper_cannot6* *pepper_cannot4* *pepper_cannot5* *reset_pose* *reset_pose*  *reset_pose* *reset_pose* *pepper_can2* *pepper_can1* *pepper_can3* *pepper_can4* *pepper_can5* *pepper_can4* *pepper_can6* *reset_pose*))

;;*pepper_watch*
;;used when pepper see something far
(setq *pepper_watch1* #f(2.0 -2.0 -5.0 100.0 50.0 -20.0 -80.0 -40.0 -60.0 -10.0 -10.0 60.0 40.0 0.0 0.0))
(setq *pepper_watch2* #f(2.0 10.0 0.0 100.0 50.0 -20.0 -80.0 -40.0 -60.0 -10.0 -10.0 60.0 40.0 0.0 0.0))
(setq *pepper_watch3* #f(2.0 -10.0 0.0 100.0 50.0 -20.0 -80.0 -40.0 -60.0 -10.0 -10.0 60.0 40.0 0.0 0.0))

;; *pepper_run*
;;used when introducing her 3D camera
(setq *pepper_run* (list *pepper_watch2* *pepper_watch1*  *pepper_watch3* *pepper_watch1* *pepper_watch2* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *pepper_open_hand* *pepper_run1* *pepper_run2* *pepper_run1*))

;; 
(setq *hope1* #f(2.0 -2.0 -5.0 30.0 10.0 -70.0 -70.0 -40.0 30.0 -10.0 70.0 70.0 40.0 0.0 0.0))
(setq *hope2* #f(2.0 -2.0 -5.0 20.0 10.0 -70.0 -70.0 -40.0 20.0 -10.0 70.0 70.0 40.0 0.0 10.0))
;;lift two hands beside face and bow slightly to right
(setq *lift_two_hands_beside_face_and_bow_slightly_to_right* #f(2.0 -20.0 -20.0 -40.0 40.0 -40.0 -89.5 0.0 -40.0 -40.0 40.0 89.5 0.0 0.0 -25.0))
;;bow slightly to left
(setq *bow_slightly_to_left* #f(2.0 20.0 -20.0 -40.0 40.0 -40.0 -89.5 0.0 -40.0 -40.0 40.0 89.5 0.0 0.0 -25.0))

;;show the hope
;;put two hands in front of the body
(setq *put_two_hands_in_front_of_the_body* #f(2.81249 -0.703127 0.351563 20.0 40.957 -5.0 -89.5 -90.0 57.5684 -20.5664 20.0 80.0 27.5956 0.0 -5.0))
;;open two arms widely and look at the sky 
(setq *open_two_arms_widely_and_look_at_the_sky* #f(2.81249 -0.703127 0.351563 -39.9902 40.957 -100.02 -10.0195 -1.32078 57.5684 -20.5664 80.5957 15.7324 27.5956 0.0 -24.9609))

;; *look_around_hope*
(setq *look_around_hope* (list *lift_two_hands_beside_face_and_bow_slightly_to_right* *bow_slightly_to_left* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *reset_pose* *put_two_hands_in_front_of_the_body* *open_two_arms_widely_and_look_at_the_sky* *open_two_arms_widely_and_look_at_the_sky*  *reset_pose*))


