#!/usr/bin/env roseus
;; initialpose_publisher.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "geometry_msgs")
(ros::roseus "initialpose_publisher")
(ros::rate 1)

(defparameter *transform-base* (ros::get-param "~transform_base" "map"))
(defparameter *transform-frame* (ros::get-param "~transform_frame" "eng2/7f/73B2"))
(defparameter *initial-pose-x* (* (ros::get-param "~initial_pose_x" 0.0) 1000.0))
(defparameter *initial-pose-y* (* (ros::get-param "~initial_pose_y" 0.0) 1000.0))
(defparameter *initial-pose-yaw* (ros::get-param "~initial_pose_yaw" 0.0))
(defparameter *amcl-pose* nil)

(defun make-initial-pose (x y yaw &optional (frame-id "map"))
  (unless (boundp '*tfl*)
    (setq *tfl* (instance ros::transform-listener :init)))
  (unless (send *tfl* :wait-for-transform *transform-base* frame-id (ros::time 0) 30)
    (ros::ros-error "failed to transform ~A -> ~A" *transform-base* frame-id)
    (return-from make-initial-pose nil))
  (let ((msg (instance geometry_msgs::PoseWithCovarianceStamped :init))
        (trans (send *tfl* :lookup-transform *transform-base* frame-id (ros::time 0)))
        (pose (make-coords :pos (float-vector x y 0)
                           :rpy (list yaw 0 0))))
  (ros::ros-info "publish initialpose ~A ~A" frame-id pose)
  (setq pose (send (send trans :copy-worldcoords) :transform pose))
  (send msg :header :stamp (ros::time-now))
  (send msg :header :frame_id *transform-base*)
  (send msg :pose
        (instance geometry_msgs::PoseWithCovariance :init
                  :pose (ros::coords->tf-pose pose)
                  :covariance (float-vector 0.25 0.0 0.0 0.0 0.0 0.0 0.0 0.25 0.0 0.0
                                            0.0  0.0 0.0 0.0 0.0 0.0 0.0 0.0  0.0 0.0
                                            0.0  0.0 0.0 0.0 0.0 0.0 0.0 0.0  0.0 0.0
                                            0.0  0.0 0.0 0.0 0.0 0.06853891945200942)))
  msg))

(defun amcl-pose-cb (msg)
  (setq *amcl-pose* msg))

(defun main ()
  (let (msg amcl-cds init-cds cds-diff)
    (ros::advertise "/initialpose" geometry_msgs::PoseWithCovarianceStamped 1 t)
    (unix:sleep 1)
    (ros::subscribe "/amcl_pose" geometry_msgs::PoseWithCovarianceStamped #'amcl-pose-cb)

    (setq msg
          (make-initial-pose *initial-pose-x*
                             *initial-pose-y*
                             *initial-pose-yaw*
                             *transform-frame*))
    (dotimes (i 30)
      (ros::spin-once)
      (ros::sleep)

      (when *amcl-pose*
        (setq init-cds (ros::tf-pose->coords (send msg :pose :pose)))
        (setq amcl-cds (ros::tf-pose->coords (send *amcl-pose* :pose :pose)))
        (setq cds-diff (+ (norm (send init-cds :difference-position amcl-cds))
                          (norm (send init-cds :difference-rotation amcl-cds))))
        (ros::ros-info "error of /initialpose and /amcl_pose: ~A" cds-diff)
        (when (< cds-diff 100)
          (return-from main t))
        (ros::publish "/initialpose" msg)))
    (ros::ros-error "failed to change initial pose")))

(main)
(exit)
