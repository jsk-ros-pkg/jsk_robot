#!/usr/bin/env roseus
(ros::roseus "lookaround")
(require "models/room73b2-scene.l")
(load "package://pr2eus/pr2-interface.l")

(ros::load-ros-manifest "geometry_msgs")
(pr2-init)
(room73b2)

(unless (boundp '*tfl*) (setq *tfl* (instance transform-listener :init)))
(if (not (boundp '*tfb*)) (setq *tfb* (instance ros::transform-broadcaster :init)))

(defun sync-pr2-pos-73b2 ()
  (setq pr2-trans (send *tfl* :lookup-transform "/eng2/7f/73B2" "/base_footprint" (ros::time 0)))
  (send *pr2* :transform pr2-trans :world)
  )
(sync-pr2-pos-73b2)

(defun pr1012-pose-callback(msg)
  (let ((pr1012-head-pose (ros::tf-pose-stamped->coords msg)))
    (send *tfb* :send-transform pr1012-head-pose (send msg :header :frame_id) "pr1012_head_tilt_link")
    (send *pr2* :head :look-at (send pr1012-head-pose :worldpos))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *irtviewer* :draw-objects)
    ))

(objects (list *pr2* *room73b2*))

(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(ros::subscribe "/pr1012_head_pos_from_73b2" geometry_msgs::PoseStamped #'pr1012-pose-callback)

(ros::spin)
;; (while (ros::ok)
;;   (let* ((new-head-pose (send *tfl* :lookup-transform "/eng2/7f/73B2" "/head_tilt_link" (ros::time 0)))
;;          (pr1012-head-pose-msg (ros::coords->tf-pose-stamped (make-coords :coords new-head-pose) "/eng2/7f/73B2")))
;;     ;;(sync-pr2-pos-73b2)
;;     (ros::publish "/pr1012_head_pos_from_73b2" pr1012-head-pose-msg)
;;     (send *irtviewer* :draw-objects)
;;     )
;;   )
