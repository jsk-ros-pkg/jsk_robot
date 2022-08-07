#!/usr/bin/env roseus                                                                               \

(ros::load-ros-manifest "roseus")
(ros::roseus "unitree-scared")
;;(load "package://unitreeeus/unitree-interface.l")


(defun scared-pose-1(&optional (time 3000))
  (progn (send *go1* :angle-vector (send *ri* :state :potentio-vector))
         (send *ri* :body-pose (coords :pos #f(0 0 -90)))
         ))

(defun scared-pose-2(&optional (time 3000))
  (progn (send *go1* :angle-vector (send *ri* :state :potentio-vector))
         (send *ri* :body-pose '(0 0.3 -0.2))
         ))

(defun reset-pose(&optional (time 3000))
  (progn (send *go1* :angle-vector (send *ri* :state :potentio-vector))
         (send *go1* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0))
         (send *ri* :body-pose '(0 0 0))
         ))

(defun scared-main()
  (print "scared")
  ;;(go1-init)
  (reset-pose 1000)
  (unix:usleep(* 1000 1000))
  (scared-pose-1 800)
  (unix:usleep(* 1000 1000))
  (scared-pose-2 800)
  (unix:usleep(* 1000 2000))
  (reset-pose)
)
