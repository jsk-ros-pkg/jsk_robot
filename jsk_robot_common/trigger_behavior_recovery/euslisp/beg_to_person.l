#!/usr/bin/env roseus

;;
;; Beg to person trigger behavior server
;;
;; This action is to beg to a person on a path when to make way for a robot.
;; Currently conditions below are assumed.
;;
;;  - robot must be Fetch
;;  - boundin box array of people topic must exist
;;

;;
;; ROS Interfaces
;;
;;   Subscribers
;;
;;     * `~path` (type: nav_msgs::Path)
;;       Path to check if a person stands near
;;
;;     * `~person_bbox_array` (type: jsk_recognition_msgs::BoundingBoxArray)
;;       Bounding box array of people.
;;
;;   Action Server
;;
;;     * `~behavior` (type: trigger_behavior_msgs::TriggerBehaviorAction)
;;       Trigger behavior action server
;;
;;   Parameters
;;
;;     * `~odom_frame_id` (type: string)
;;       Odometry frame ID
;;
;;     * `~speak_text` (type: string)
;;       Speeched text when person on path
;;


(ros::load-ros-manifest "trigger_behavior_msgs")
(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")

(ros::roseus "beg_to_person")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))
(setq *speak-text* (ros::get-param "~speak_text" "道をあけて下さい"))
(setq *path-coords-list* nil)
(setq *people-coords-list* nil)

(defun check-person-on-path (person-coords path-coords-list &key (threshold 2000))
  (let (distance)
    (setq distance (calc-distance-person-to-path person-coords path-coords-list))
    (ros::ros-info "distance: ~A" distance)
    (< distance threshold)))

(defun calc-distance-person-to-path (person-coords path-coords-list)
  (let (nearest-distance temp-distance)
    (dolist (path-coords path-coords-list)
      (setq temp-distance (distance (send path-coords :pos) (send person-coords :pos)))
      (if (not nearest-distance)
          (setq nearest-distance temp-distance)
        (progn
          (if (< temp-distance nearest-distance)
              (setq nearest-distance temp-distance)
            )
          )
        )
      )
    nearest-distance))

(defun get-nearest-person-to-path (people-coords-list path-coords-list)
  (let (nearest-person-coords nearest-distance temp-distance)
    (dolist (person-coords people-coords-list)
      (setq temp-distance
              (calc-distance-person-to-path
                person-coords
                path-coords-list))
      (if (not nearest-distance)
        (progn
          (setq nearest-distance temp-distance)
          (setq nearest-person-coords person-coords)
          ))
      (if (< temp-distance nearest-distance)
        (progn
          (setq nearest-distance temp-distance)
          (setq nearest-person-coords person-coords)
          ))
      )
    nearest-person-coords))

(defun transform-coords-fixedbased-to-robotbased (coords-odombased)
  (let (coords-robotbased
        (coords-fixed-to-robot
           (send *tfl* :lookup-transform
                 *fixed-frame-id*
                 *base-frame-id*
                 (ros::time)))
        )
    (setq coords-robotbased
          (transform-coords
            (send coords-fixed-to-robot :inverse-transformation)
            coords-odombased))
    coords-robotbased))

(defun look-at-target-coords (target-coords)
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :look-at-target target-coords)
  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 500)
  )

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

(defun execute-cb (server goal)
  (let (result nearest-person-coords)
    (ros::ros-warn "server called")
    (setq result (send server :result))
    (if (or (not *path-coords-list*)
            (not *people-coords-list*))
      (progn
        (send result :result :success nil)
        (send result :result :message "No message cached.")
        (send server :set-aborted result)
        (return-from execute-cb)
        ))
    (setq nearest-person-coords
          (get-nearest-person-to-path
            *people-coords-list*
            *path-coords-list*))
    (ros::ros-info "nearest-person-coords: ~A" nearest-person-coords)
    (if (check-person-on-path
          nearest-person-coords
          *path-coords-list*)
      (progn
        (look-at-target-coords (transform-coords-fixedbased-to-robotbased nearest-person-coords))
        (send *ri* :speak-jp *speak-text*)
        (send result :result :success t)
        (send result :result :message "Beg to person to make way.")
        (ros::ros-info "Beg to person to make way.")
        )
      (progn
        (send result :result :success t)
        (send result :result :message "No person on path.")
        (ros::ros-info "No person on path.")
        )
      )
    (send server :set-succeeded result)
    ))

(ros::subscribe "~path" nav_msgs::Path #'callback-path)
(ros::subscribe "~person_bbox_array" jsk_recognition_msgs::BoundingBoxArray #'callback-bbox-array)
(setq *as*
      (instance ros::simple-action-server :init
                  "~behavior"
                  trigger_behavior_msgs::TriggerBehaviorAction
                  :execute-cb #'execute-cb))

(ros::ros-info "initialized")

(ros::rate 500)
(while (ros::ok)
 (send *as* :worker)
 (ros::spin-once)
 (ros::sleep))
