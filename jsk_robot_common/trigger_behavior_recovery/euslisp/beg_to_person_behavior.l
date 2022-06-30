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
;;   Publisher
;;
;;    * `~debug` (type: geometry_msgs::PoseStamped)
;;       target object pose to look at
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
(ros::load-ros-manifest "coral_usb")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://trigger_behavior_recovery/euslisp/utils.l")

(ros::roseus "beg_to_person")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))
(setq *speak-text* (ros::get-param "~speak_text" "道をあけて下さい"))
(setq *use-panorama-coral-detector* (ros::get-param "~use_panorama_coral_detector" nil))
(if *use-panorama-coral-detector*
  (progn
    (setq *panorama-edgetpu-node-name* (ros::get-param "~panorama_edgetpu_node_name" "edgetpu_panorama_object_detector"))
    ))

(setq *path-coords-list* nil)
(setq *people-coords-list* nil)

(defun transform-coords-fixedbased-to-robotbased (coords-fixedbased)
  (let (coords-robotbased
        (coords-robot-to-fixed
           (send *tfl* :lookup-transform
                 *base-frame-id*
                 *fixed-frame-id*
                 (ros::time)))
        )
    (setq coords-robotbased (transform-coords coords-robot-to-fixed coords-fixedbased))
    coords-robotbased))

(defun look-at-target-coords (target-coords)
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :look-at-target target-coords)
  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000)
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

(defun publish-debug-pose (coords)
  (let ((msg (instance geometry_msgs::PoseStamped :init)))
    (send msg :header :frame_id *fixed-frame-id*)
    (send msg :header :stamp (ros::time-now))
    (send msg :pose (ros::coords->tf-pose coords))
    (ros::publish "~debug" msg)
    )
  )

(defun execute-cb (server goal)
  (ros::ros-warn "Action server called")
  (if *use-panorama-coral-detector*
    (let (ret res result (req (instance coral_usb::StartNodeRequest :init)))
      (send req :name *panorama-edgetpu-node-name*)
      (setq ret (ros::wait-for-service "~start_edgetpu_node" :timeout 5.0))
      (if (not ret)
        (progn
          (send result :result :success nil)
          (send result :result :message "Failed to find edgetpu manager interface.")
          (ros::ros-error "Failed to find edgetpu manager interface.")
          (send server :set-aborted result)
          (return-from execute-cb)
          )
        )
      (setq res (ros::service-call "~start_edgetpu_node" req t))
      (if (not (send res :success))
        (progn
          (send result :result :success nil)
          (send result :result :message "Failed to start edgetpu node.")
          (ros::ros-error "Failed to start edgetpu node.")
          (send server :set-aborted result)
          (return-from execute-cb)
          )
        )
      )
    )
  (let (nearest-person-coords
        target-person-coords
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
    (setq nearest-person-coords (get-nearest-object-to-path *people-coords-list* *path-coords-list*))
    (publish-debug-pose nearest-person-coords)
    (if (check-object-on-path nearest-person-coords *path-coords-list*)
      (progn
        (setq target-person-coords (transform-coords-fixedbased-to-robotbased nearest-person-coords))
        (look-at-target-coords target-person-coords)
        (send *ri* :speak-jp *speak-text* :wait t)
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
(ros::advertise "~debug" geometry_msgs::Posestamped 1)
(if *use-panorama-coral-detector*
  (progn
    ))
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
