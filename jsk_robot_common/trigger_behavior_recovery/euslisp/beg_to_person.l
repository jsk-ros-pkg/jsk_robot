#!/usr/bin/env roseus

(ros::load-ros-manifest "trigger_behavior_msgs")
(ros::load-ros-manifest "nav_msgs")
(load "package://fetcheus/fetch-interface.l")

(ros::roseus "beg_to_person")

(fetch-init)

(setq *path* nil)
(defun callback-path (msg)
  (setq *path* msg)
  )

(defun execute-cb (server goal)
  (let (result)
    (setq result (send server :result))
    (send server :set-succeeded result)
    ))

(setq *as*
      (instance ros::simple-action-server :init
                  "~behavior"
                  trigger_behavior_msgs::TriggerBehaviorAction
                  :execute-cb #'execute-cb))
(ros::subscribe "~path" nav_msgs::Path callback-path)

(ros::rate 500)
(while (ros::ok)
 (send s :worker)
 (ros::spin-once)
 (ros::sleep))
