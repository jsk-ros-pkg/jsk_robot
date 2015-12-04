#!/usr/bin/env roseus
(ros::roseus "lookaround_base")
(require "models/room73b2-scene.l")
(load "package://pr2eus/pr2-interface.l")

(setq thres 1000)
(setq between-thres 3000)

(ros::load-ros-manifest "geometry_msgs")
(pr2-init)
(room73b2)

(unless (boundp '*tfl*) (setq *tfl* (instance transform-listener :init)))
(unless (boundp '*tfb*) (setq *tfb* (instance ros::transform-broadcaster :init)))

(defun sync-pr2-pos-73b2 ()
  (setq pr2-trans (send *tfl* :lookup-transform "/eng2/7f/73B2" "/base_footprint" (ros::time 0)))
  (when pr2-trans
    (send *pr2* :reset-coords)
    (send *pr2* :transform pr2-trans))
  )
(sync-pr2-pos-73b2)


;;(setq pr1012-base-trajectories (list))
(setq prev-pr1012-pos nil)

(defun pr1012-pose-callback(msg)
  (let ((pr1012-head-pose (ros::tf-pose-stamped->coords msg)))
    (send *tfb* :send-transform pr1012-head-pose (send msg :header :frame_id) "pr1012_head_tilt_link")
    (setq pr1012-base-pos (send pr1012-head-pose :pos))
    (setf (elt pr1012-base-pos 2) 0)

    ;;Update PR1012
    ;; (if (or (eq (length pr1012-base-trajectories) 0) (> (norm (v- pr1012-base-pos (car (last pr1012-base-trajectories)))) thres))
    ;;     (setq pr1012-base-trajectories (append pr1012-base-trajectories (list pr1012-base-pos))))


    ;;First Rot to Robot
    (setq pr1040->pr1012 (send *pr2* :transformation pr1012-head-pose))
    (send *tfb* :send-transform pr1040->pr1012 "base_footprint" "pr1012_head_tilt_link3")
    (setq target-rot-deg (rad2deg (atan (/ (elt (send pr1040->pr1012 :pos) 1)
                                           (elt (send pr1040->pr1012 :pos) 0)))))
    (ros::ros-info "pr1040 ~A " (send *pr2* :pos))
    (ros::ros-info "pr1012 ~A " pr1012-base-pos)
    (ros::ros-info "Target x:~A y:~A deg: ~A x/y: ~A" (elt (send pr1040->pr1012 :pos) 0)
                   (elt (send pr1040->pr1012 :pos) 1) target-rot-deg (/ (elt (send pr1040->pr1012 :pos) 1)
                                                                        (elt (send pr1040->pr1012 :pos) 0)))
    (send *ri* :go-pos-unsafe 0 0 target-rot-deg)
    (sync-pr2-pos-73b2)


    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;;If So far
    ;; (when (> (norm (v- pr1012-base-pos (send *pr2* :pos))) between-thres)
    ;;   (send *ri* :go-pos-unsafe (elt pr1012-base-pos 0) (elt pr1012-base-pos 1) 0)

    ;;   ;;After Move , Rot
    ;;   (setq pr2-trans (send *tfl* :lookup-transform "/eng2/7f/73B2" "/base_footprint" (ros::time 0)))
    ;;   (send *pr2* :transform pr2-trans :world)
    ;;   (setq pr1040->pr1012 (send *pr2* :transformation pr1012-head-pose))
    ;;   (setq target-rot-deg (rad2deg (atan (/ (elt (send pr1040->pr1012 :pos) 0)
    ;;                                          (elt (send pr1040->pr1012 :pos) 1)))))
    ;;   (send *ri* :go-pos-unsafe 0 0 target-rot-deg)
    ;;   )

    ;; (when (or (not nil) (> (norm (v- pr1012-base-pos prev-pr1012-pos)) thres))
    ;;   (setq prev-pr1012-pos pr1012-base-pos)
    ;;   )


    ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    ;;(send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    ))

(objects (list *pr2* *room73b2*))

;;(init-pose)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(ros::subscribe "/pr1012_head_pos_from_73b2" geometry_msgs::PoseStamped #'pr1012-pose-callback)

(ros::rate 10)
(ros::spin)
