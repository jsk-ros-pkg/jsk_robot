#!/usr/bin/env roseus

(ros::load-ros-manifest "trigger_behavior_msgs")
(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")

(ros::roseus "beg_to_person")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *odom-frame-id* (ros::get-param "~odom_frame_id" "odom"))
(setq *speak-text* (ros::get-param "~speak_text" "道をあけて下さい"))
(setq *path-coords-list* nil)
(setq *people-coords-list* nil)

(defun check-person-on-path (person-coords path-coords-list &key (threshold 1.0))
  (let (distance)
    (setq distance (calc-distance-person-to-path
                     person-coords path-coords-list))
    (< distance threshold)))

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
  (let (nearest-person-coords nearest-distance temp-distance)
    (dolist (person-coords people-coords-list)
      (setq temp-distance
              (calc-distance-person-to-path
                person-coords
                path-coords-list))
      (if (not nearest-distance)
        (setq nearest-distance temp-distance))
      (if (< temp-distance nearest-distance)
        (setq nearest-distance temp-distance))
      )
    nearest-person-coords))

(defun transform-coords-odombased-to-robotbased (coords-odombased)
  (let (coords-robotbased
        (coords-odom-to-robot (send *ri* :state :odom :pose)))
    (setq coords-robotbased
          (transform-coords
            (send coords-odom-to-robot :inverse-transformation)
            coords-odombased))
    coords-robotbased))

(defun look-at-target-coords (target-coords)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :look-at-target target-coords)
  (send *ri* :angle-vector-raw (send *robot* :angle-vector) 500)
  )

(defun callback-path (msg)
  (let* ((coords-list nil)
         (coords-odom-to-path-frame
           (send *tfl* :lookup-transform
                 (send msg :header :frame_id)
                 *odom-frame-id*
                 (ros::time)))
         )
    (if coords-odom-to-path-frame
      (setq *path-coords-list*
          (mapcar
            (lambda (ros-pose-on-path-frame)
              (transform-coords
                coords-odom-to-path-frame
                (ros::tf-pose->coords (send ros-pose-on-path-frame :pose))
                )
              )
            (send msg :poses))))
    ))

(defun callback-bbox-array (msg)
  (let* ((coords-list nil)
         (coords-odom-to-bbox-frame
           (send *tfl* :lookup-transform
                 (send msg :header :frame_id)
                 *odom-frame-id*
                 (ros::time)))
         )
    (if coords-odom-to-bbox-frame
      (setq *people-coords-list*
          (mapcar
            (lambda (bbox)
              (transform-coords
                coords-odom-to-bbox-frame
                (ros::tf-pose->coords (send bbox :pose))
                )
              )
            (send msg :boxes))))
    ))

(defun execute-cb (server goal)
  (let (result nearest-person-coords)
    (setq result (send server :result))
    (setq nearest-person-coords
          (get-nearest-person-to-path
            *people-coords-list*
            *path-coords-list*))
    (if (check-person-on-path
          nearest-person-coords
          *path-coords-list*)
      (progn
        (look-at-target-coords (transform-coords-odombased-to-robotbased nearest-person-coords))
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
