#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")
(ros::load-ros-manifest "seedhand_std")

(defclass aero-hand-interface
  :super propertied-object
  :slots (service-name)
  )
(defmethod aero-hand-interface
  (:init
   (&key (name "/aero_hand_controller"))
   (ros::roseus "hand_interface")
   (setq service-name name)
   (ros::wait-for-service service-name)
   )
  (:grasp (arm &key (tfail 0.8) (twarn -0.9) (power))
    (let (req res)
      (setq req (instance
                 aero_startup::HandControlRequest
                 :init
                 :hand (cond ((eq arm :larm) aero_startup::HandControlRequest::*HAND_LEFT*)
                             ((eq arm :rarm) aero_startup::HandControlRequest::*HAND_RIGHT*)
                             ((eq arm :arms) aero_startup::HandControlRequest::*HAND_BOTH*))
                 :command aero_startup::HandControlRequest::*COMMAND_GRASP*
                 ;; :power
                 :thre_fail tfail
                 :thre_warn twarn
                 ))
      (setq res (ros::service-call service-name req))
      ;;(print (send res :status))
      ))
  (:ungrasp (arm &key (power))
    (let (req res)
      (setq req (instance
                 aero_startup::HandControlRequest
                 :init
                 :hand (cond ((eq arm :larm) aero_startup::HandControlRequest::*HAND_LEFT*)
                             ((eq arm :rarm) aero_startup::HandControlRequest::*HAND_RIGHT*)
                             ((eq arm :arms) aero_startup::HandControlRequest::*HAND_BOTH*))
                 :command aero_startup::HandControlRequest::*COMMAND_UNGRASP*
                 ;; :power
                 ))
      (setq res (ros::service-call service-name req))
      ;;(print (send res :status))
      ))
  (:grasp-angle (arm angle)
;; rarm :  open-limit 50 close-limit -50
;; larm :  open-limit -50  close-limit 50
    (let (req res)
      (setq req (instance
                 aero_startup::HandControlRequest
                 :init
                 :hand (cond ((eq arm :larm) aero_startup::HandControlRequest::*HAND_LEFT*)
                             ((eq arm :rarm) aero_startup::HandControlRequest::*HAND_RIGHT*)
                             ;;((eq arm :arms) aero_startup::HandControlRequest::*HAND_BOTH*)
                             )
                 :command aero_startup::HandControlRequest::*COMMAND_GRASP_ANGLE*
                 ;; :power
                 :larm_angle angle
                 :rarm_angle angle
                 ))
      (setq res (ros::service-call service-name req))
      ;;(print (send res :status))
      ))
  )
