#!/usr/bin/env roseus
(ros::roseus "lookaround_base")
(require "models/room73b2-scene.l")
(load "package://pr2eus/pr2-interface.l")

(setq thres 1500)
(setq between-thres 2500)

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

(setq prev-pr1012-pos nil)
(defun pr1012-pose-callback(msg)
  (let ((pr1012-head-pose (ros::tf-pose-stamped->coords msg)))
    (send *tfb* :send-transform pr1012-head-pose (send msg :header :frame_id) "pr1012_head_tilt_link")
    (setq pr1012-base-pos (send pr1012-head-pose :pos))
    (setf (elt pr1012-base-pos 2) 0)


    (if (and (eq (elt pr1012-base-pos 0) 0)
	     (eq (elt pr1012-base-pos 1) 0)
	     (eq (elt pr1012-base-pos 2) 0))
	(return-from pr1012-pose-callback nil))
    ;;First Rot to Robot
    (setq pr1040->pr1012 (send *pr2* :transformation pr1012-head-pose))
    (setq target-rot-deg (rad2deg (atan (/ (elt (send pr1040->pr1012 :pos) 1)
                                           (elt (send pr1040->pr1012 :pos) 0)))))

    (send *ri* :go-pos-unsafe 0 0 target-rot-deg)
    (sync-pr2-pos-73b2)


    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;;If So far
    (when (> (norm (v- pr1012-base-pos (send *pr2* :pos))) between-thres)
      (ros::ros-error "Too FAR!!")
      (if prev-pr1012-pos
	  (progn
	    (setq pr1040->prev-pr1012 (send *pr2* :transformation (make-coords :pos prev-pr1012-pos)))
	    (setq goal-pos (send pr1040->prev-pr1012 :pos))
	    (send *tfb* :send-transform (make-coords :pos goal-pos) "base_footprint" "goal_with_prev")
	    (send *ri* :go-pos  (/ (elt goal-pos 0) 1000) (/ (elt goal-pos 1) 1000) 0))
	(progn
	  ;;                                 (thres length)
	  ;;|--------goal-diff--------|--------move-diff-----|
	  ;;PR1040 ------------------>GOAL------------------>PR1012
	  (setq diff (send pr1040->pr1012 :pos))
	  (setq norm-diff (normalize-vector diff))
	  (setq move-diff (scale thres norm-diff))
	  (setq goal-diff (v- diff move-diff))
	  (send *tfb* :send-transform (make-coords :pos goal-diff) "base_footprint" "goal")
	  (send *ri* :go-pos (/ (elt goal-diff 0) 1000) (/ (elt goal-diff 1) 1000) 0)))
    
      )
    
    (when (or (not prev-pr1012-pos) (> (norm (v- pr1012-base-pos prev-pr1012-pos)) thres))
      (setq prev-pr1012-pos pr1012-base-pos)
      )
    (send *irtviewer* :draw-objects)
    ))

(objects (list *pr2* *room73b2*))

(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(use-tilt-laser-obstacle-cloud nil)
(change-inflation-range 0.3)
(ros::subscribe "/pr1012_head_pos_from_73b2" geometry_msgs::PoseStamped #'pr1012-pose-callback)
(ros::rate 10)
(ros::spin)
