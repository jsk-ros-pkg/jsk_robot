#!/usr/bin/env roseus

(ros::load-ros-manifest "trigger_behavior_msgs")
(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")

(ros::roseus "beg_to_person")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *speak-text* (ros::get-param "~speak_text" "道をあけて下さい"))
(setq *path-coords-list* nil)
(setq *people-coords-list* nil)

(defun check-person-on-path (bbox-array path)
  ;; TODO
  )

(defun calc-distance-person-to-path (person-coords path-coords-list)
  (let (nearest-distance)
    (dolist (path-coords path-coords-list)
      (if (not nearest-distance)
          (setq nearest-distance (norm (v- (send path-coords :pos) (send person-coords :pos))))
        (progn
          (if (< (norm (v- (send path-coords :pos) (send person-coords :pos)))
                 nearest-distance)
            (setq nearest-distance (norm (v- (send path-coords :pos) (send person-coords :pos))))
            ))))
    nearest-distance))

(defun get-nearest-person-to-path (people-coords-list path-coords-list)
  (let (nearest-person-coords nearest-distance)
    (dolist (person-coords people-coords-list)
      )
    nearest-person-coords))

(defun look-at-bbox (bbox)
  )

(defun callback-path (msg)
  (let* ((coords-list nil)
         (coords-fixed-to-path-frame
           (send *tfl* :lookup-transform
                 (send msg :header :frame_id)
                 *fixed-frame-id*
                 (ros::time)))
         )
    (if coords-fixed-to-path-frame
      (setq *path-coords-list*
          (mapcar
            (lambda (ros-pose-on-path-frame)
              (transform-coords
                coords-fixed-to-path-frame
                (ros::tf-pose->coords (send ros-pose-on-path-frame :pose))
                )
              )
            (send msg :poses))))
    ))

(defun callback-bbox-array (msg)
  (let* ((coords-list nil)
         (coords-fixed-to-bbox-frame
           (send *tfl* :lookup-transform
                 (send msg :header :frame_id)
                 *fixed-frame-id*
                 (ros::time)))
         )
    (if coords-fixed-to-bbox-frame
      (setq *people-coords-list*
          (mapcar
            (lambda (bbox)
              (transform-coords
                coords-fixed-to-bbox-frame
                (ros::tf-pose->coords (send bbox :pose))
                )
              )
            (send msg :boxes))))
    ))

(defun execute-cb (server goal)
  (let (bbox-array result target-bbox)
    (setq result (send server :result))
    (setq bbox-array (get-person-bbox-array))
    (if (check-person-on-path bbox-array *path-coords-list*)
      (progn
        (setq target-bbox (get-nearest-person-to-path bbox-array *path-coords-list*))
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

(ros::rate 500)
(while (ros::ok)
 (send s :worker)
 (ros::spin-once)
 (ros::sleep))
