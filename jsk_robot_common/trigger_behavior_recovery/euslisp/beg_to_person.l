#!/usr/bin/env roseus

(ros::load-ros-manifest "trigger_behavior_msgs")
(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")

(ros::roseus "beg_to_person")
(fetch-init)

(setq *speak-text* (ros::get-param "~speak_text" "道をあけて下さい"))

(defun get-person-bbox-array ()
  (let (bbox-array)
    (setq bbox-array (ros::one-shot-subscribe
                          "~bbox_array"
                          jsk_recognition_msgs::BoundingBoxArray
                          :timeout 10))
    bbox-array))

(defun check-person-on-path (bbox-array path)
  ;; TODO
  )

(defun get-nearest-person-to-path (bbox-array path)
  ;; TODO
  )

(defun look-at-bbox (bbox)
  
  )

(setq *path* nil)
(defun callback-path (msg)
  (setq *path* msg)
  )

(setq *costmap* nil)
(defun callback-costmap (msg)
  (setq *costmap* msg)
  )

(defun execute-cb (server goal)
  (let (bbox-array result target-bbox)
    (setq result (send server :result))
    (setq bbox-array (get-person-bbox-array))
    (if (check-person-on-path bbox-array *path*)
      (progn
        (setq target-bbox (get-nearest-person-to-path bbox-array *path*))
        (look-at-bbox target-bbox)
        (send *ri* :speak-jp *speak-text*)
        (send result :success t)
        (send result :message "Beg to person to make way.")
        )
      (progn
        (send result :success t)
        (send result :message "No person on path.")
        )
      )
    (send server :set-succeeded result)
    ))

(setq *as*
      (instance ros::simple-action-server :init
                  "~behavior"
                  trigger_behavior_msgs::TriggerBehaviorAction
                  :execute-cb #'execute-cb))
(ros::subscribe "~path" nav_msgs::Path callback-path)
(ros::subscribe "~costmap" nav_msgs::OccupancyGrid callback-costmap)

(ros::rate 500)
(while (ros::ok)
 (send s :worker)
 (ros::spin-once)
 (ros::sleep))
