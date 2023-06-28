#!/usr/bin/env roseus

;;
;; Move chair trigger behavior server
;;


(ros::load-ros-manifest "trigger_behavior_msgs")
(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://trigger_behavior_recovery/euslisp/utils.l")

(ros::roseus "move_chaier_out_of_the_way")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))

(defun look-at-target-coords (target-coords)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *robot* :look-at-target target-coords)
  (send *ri* :angle-vector-raw (send *robot* :angle-vector) 500)
  )

(setq *path-coords-list* nil)
(setq *bbox-coords-list* nil)

(defun callback-path (msg)
  (let* ((coords-list nil)
         (coords-fixed-to-path-frame
           (send *tfl* :lookup-transform
                 *fixed-frame-id*
                 (send msg :header :frame_id)
                 (ros::time)))
         )
    (if coords-fixed-to-path-frame
      (setq *path-coords-list*
          (mapcar
            #'(lambda (ros-pose-on-path-frame)
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
                 *fixed-frame-id*
                 (send msg :header :frame_id)
                 (ros::time)))
         )
    (if coords-fixed-to-bbox-frame
      (setq *people-coords-list*
          (mapcar
            #'(lambda (bbox)
              (transform-coords
                coords-fixed-to-bbox-frame
                (ros::tf-pose->coords (send bbox :pose))
                )
              )
            (send msg :boxes))))
    ))

(defun move-away-chair (target-coords)
  ;; TODO implement this behavior
  t)

(defun execute-cb (server goal)
  (ros::ros-warn "server called")
  (let (nearest-bbox-coords
        target-bbox-coords
        (result (send server :result))
        )
    (if (or (not *path-coords-list*) (not *people-coords-list*))
      (progn
        (send result :result :success nil)
        (send result :result :message "No message cached yet.")
        (ros::ros-error "No message cached yet.")
        (send server :set-aborted result)
        (return-from execute-cb)
        ))
    (setq nearest-bbox-coords (get-nearest-object-to-path *bbox-coords-list* *path-coords-list*))
    (if (check-object-on-path nearest-bbox-coords *path-coords-list*)
      (progn
        (setq target-bbox-coords (transform-coords-odombased-to-robotbased nearest-person-coords))
        (if (move-away-chair target-bbox-coords)
          (progn
            (send result :success t)
            (send result :message "Succeeded to move away chair")
            (ros::ros-info "Succeeded to move away chair")
            )
          (progn
            (send result :success t)
            (send result :message "Failed to move away chair")
            (ros::ros-info "Failed to move away chair")
            )
          )
        )
      (progn
        (send result :success t)
        (send result :message "No chair on path.")
        (ros::ros-info "No chair on path.")
        )
      )
    (send server :set-succeeded result)
    ))

(ros::subscribe "~path" nav_msgs::Path #'callback-path)
(ros::subscribe "~bbox_array" jsk_recognition_msgs::BoundingBoxArray #'callback-bbox-array)
(setq *as*
      (instance ros::simple-action-server :init
                  "~behavior"
                  trigger_behavior_msgs::TriggerBehaviorAction
                  :execute-cb #'execute-cb))

(ros::rate 500)
(while (ros::ok)
 (send *as* :worker)
 (ros::spin-once)
 (ros::sleep))
