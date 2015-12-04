#!/usr/bin/env roseus
(ros::roseus "lookaround")
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
    ;;(send *tfb* :send-transform pr1012-head-pose (send msg :header :frame_id) "pr1012_head_tilt_link")
    (setq pr1012-base-pos (send pr1012-head-pose :pos))
    (setf (elt pr1012-base-pos 2) 0)

    ;;Update PR1012
    ;; (if (or (eq (length pr1012-base-trajectories) 0) (> (norm (v- pr1012-base-pos (car (last pr1012-base-trajectories)))) thres))
    ;;     (setq pr1012-base-trajectories (append pr1012-base-trajectories (list pr1012-base-pos))))


    ;;Rotate Hand
    (setq pr1040-hand->pr1012 (send (send *pr2* :rarm :end-coords) :transformation pr1012-head-pose))
    ;;    (send *tfb* :send-transform pr1040-hand->pr1012 "base_footprint" "pr1012_head_tilt_link3")
    (setq target-rot-deg (rad2deg (atan (/ (elt (send pr1040-hand->pr1012 :pos) 1)
                                           (elt (send pr1040-hand->pr1012 :pos) 0)))))
    (ros::ros-info "pr1040 ~A " (send *pr2* :pos))
    (ros::ros-info "pr1012 ~A " pr1012-base-pos)
    (ros::ros-info "Target x:~A y:~A deg: ~A x/y: ~A" (elt (send pr1040-hand->pr1012 :pos) 0)
                   (elt (send pr1040-hand->pr1012 :pos) 1) target-rot-deg (/ (elt (send pr1040-hand->pr1012 :pos) 1)
                                                                             (elt (send pr1040-hand->pr1012 :pos) 0)))

    (send *ri* :update-robot-state)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
    (send *pr2* :rarm :wrist-p :joint-angle (+ (send *pr2* :rarm :wrist-p :joint-angle) target-rot-deg))

    ;; (ros::ros-info "before init")
    ;; (setq init-pose-angle-vector #f(49.9829 20.718 73.8498 105.362 -89.7311 31.7531 -15.9999 187.452 -33.2225 3.05358 -76.2937 -104.993 -170.853 -68.7201 254.583 -0.003882 -1.842))
    ;; (send *pr2* :angle-vector init-pose-angle-vector)
    ;; (ros::ros-info "before init2")
    ;; (setq copy-rarm (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    ;; (setq rend->pr1012-head (send (send *pr2* :rarm :end-coords) :transformation pr1012-head-pose))
    ;; (setq target-coords (orient-coords-to-axis copy-rarm (send rend->pr1012-head :worldpos) :x))
    ;; (send target-coords :rotate 3.141592 :y)
    ;; (send *pr2* :inverse-kinematics
    ;;       target-coords
    ;;       :revert-if-fail nil
    ;;       :rotation-axis t
    ;;       :move-target (send *pr2* :rarm :end-coords)
    ;;       :link-list (send *pr2* :link-list
    ;;                        (send *pr2* :rarm :end-coords :parent)
    ;;                        (send *pr2* :torso_lift_link_lk)))

    (sync-pr2-pos-73b2)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :objects (list *pr2* *room73b2* target-coords (send *pr2* :rarm :end-coords)))
    (send *irtviewer* :draw-objects)
    (x::window-main-one)
    ))

(defun init-pose()
  (send *ri* :stop-grasp :arms :wait t)
  (setq init-pose-angle-vector #f(44.2525 20.7987 73.7577 105.371 -89.5071 31.7531 -14.9928 187.442 -12.9724 -9.25278 -87.5118 -76.6586 -173.269 -68.8323 267.439 -17.7999 74.2702));;#f(49.9829 20.718 73.8498 105.362 -89.7311 31.7531 -15.9999 187.452 -33.2225 3.05358 -76.2937 -104.993 -170.853 -68.7201 254.583 -0.003882 -1.842))
  (send *pr2* :angle-vector init-pose-angle-vector)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
  (unix:sleep 3)
  (send *ri* :wait-interpolation)
  (send *ri* :start-grasp)
  )

(objects (list *pr2* *room73b2*))

(init-pose)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(ros::subscribe "/pr1012_head_pos_from_73b2" geometry_msgs::PoseStamped #'pr1012-pose-callback)

(ros::rate 10)
(ros::spin)
