#!/usr/bin/env roseus
;; initialpose_publisher.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "topic_tools")
(ros::roseus "initialpose_publisher")
(ros::rate 1)

(defparameter *initial-pose-topic* "/initialpose")
(defparameter *amcl-topic* "/amcl_pose")
(defparameter *transform-base* (ros::get-param "~transform_base" "map"))
(defparameter *transform-frame* (ros::get-param "~transform_frame" "eng2/7f/73B2"))
(defparameter *initial-pose-x* (* (ros::get-param "~initial_pose_x" 0.0) 1000.0))
(defparameter *initial-pose-y* (* (ros::get-param "~initial_pose_y" 0.0) 1000.0))
(defparameter *initial-pose-yaw* (ros::get-param "~initial_pose_yaw" 0.0))
(defparameter *amcl-pose* nil)

(defun explode-frame (frame)
  "eng2/7f/73B2 -> ('eng2' '7f' '73B2')"
  (reverse (lisp::explode-directory-names frame)))
(defun join-frame (frame)
  "('eng2' '7f' '73B2') -> eng2/7f/73B2"
  (let ((str (car frame)))
    (dolist (s (cdr frame))
      (setq str (concatenate string str "/" s)))
    str))

(defun set-map-tf (&optional (ns "map_tf_mux") (timeout 10))
  (let ((srv-list (format nil "~A/list" ns))
        (srv-select (format nil "~A/select" ns))
        (target (explode-frame *transform-frame*))
        (floor-list))
    (dolist (srv (list srv-list srv-select))
      (ros::ros-info "waiting for service ~A (timeout: ~A)" srv timeout)
      (unless (ros::wait-for-service srv timeout)
        (ros::ros-error "service ~A not found. Could not set initial map tf" srv)
        (return-from set-map-tf nil)))
    (setq floor-list
          (mapcar #'explode-frame
                  (send (ros::service-call srv-list (instance topic_tools::MuxListRequest :init)) :topics)))
    (dolist (floor floor-list)
      (when (and
             (<= (length floor) (length target))
             (every #'identity
                   (mapcar #'substringp (subseq target 0 (length floor)) floor)))
        (ros::service-call srv-select (instance topic_tools::MuxSelectRequest :init
                                                :topic (join-frame floor)))
        (ros::ros-info "successfully set map tf to ~A" (join-frame floor))
        (return-from set-map-tf t)))
    (ros::ros-error "target frame ~A not found in selectable list ~A"
                    target (mapcar #'join-frame floor-list))
    nil))

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
    (ros::advertise *initial-pose-topic* geometry_msgs::PoseWithCovarianceStamped 1 t)
    (unix:sleep 1)
    (ros::subscribe *amcl-topic* geometry_msgs::PoseWithCovarianceStamped #'amcl-pose-cb)

    (setq msg
          (make-initial-pose *initial-pose-x*
                             *initial-pose-y*
                             *initial-pose-yaw*
                             *transform-frame*))

    (set-map-tf)

    (dotimes (i 30)
      (ros::spin-once)
      (ros::sleep)

      (when *amcl-pose*
        (setq init-cds (ros::tf-pose->coords (send msg :pose :pose)))
        (setq amcl-cds (ros::tf-pose->coords (send *amcl-pose* :pose :pose)))
        (setq cds-diff (+ (norm (send init-cds :difference-position amcl-cds))
                          (norm (send init-cds :difference-rotation amcl-cds))))
        (ros::ros-info "error of ~A and ~A: ~A" *initial-pose-topic* *amcl-topic* cds-diff)
        (when (< cds-diff 100)
          (return-from main t))
        (ros::publish *initial-pose-topic* msg)))
    (ros::ros-error "failed to change initial pose")
    nil))

(if (main) (exit 0) (exit 1))
