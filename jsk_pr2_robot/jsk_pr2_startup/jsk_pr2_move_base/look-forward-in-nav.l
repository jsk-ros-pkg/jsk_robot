#!/usr/bin/env roseus

(ros::roseus-add-msgs "nav_msgs")
(ros::roseus-add-msgs "sound_play")
(ros::roseus-add-msgs "move_base_msgs")
(load "package://pr2eus/pr2-interface.l")

(ros::roseus "look_forward")

(setq *look-enable* t)
(setq *speak-timestamp* (ros::time 0))

;; if (ros::spin-once) is called in callback function,
;;   stack overflow will occure.
(defun global-path-cb (msg)
  (setq *msg* msg))

(defun result-cb (msg)
  (let ((av (send *ri* :state :potentio-vector)))
    (send *pr2* :angle-vector av)
    (send *pr2* :head :angle-vector #f(0 0))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
    ))

(defun look-at-front ()
  (unless (boundp '*msg*) (return-from look-at-front nil))
  (if (< 5 (send (ros::time- (ros::time-now)
			     (send *msg* :header :stamp)) :to-sec))
      (return-from look-at-front nil))
  ;;
  (let* ((po (instance geometry_msgs::PoseStamped :init
		       :header (send *msg* :header)
		       :pose (send (car (last (send *msg* :poses))) :pose)))
	 tra (av (send *pr2* :angle-vector)))
    ;; need convert ?
    (send *tfl* :wait-for-transform
	  "base_footprint" (send po :header :frame_id)
	  (send po :header :stamp) 1)
    (setq tra (send *tfl* :transform-pose "base_footprint" po))
    (when (not tra)
      (ros::ros-error "not transform")
      (return-from look-at-front nil))
    (ros::ros-info "msg received ~A~%" *msg* (norm (send tra :worldpos)))

    (when *look-enable*
      (if (< 500 (norm (send tra :worldpos)))
	  (send *pr2* :head :look-at (v+ #f(0 0 1300) (send tra :worldpos)))
	(send *pr2* :head :look-at #f(0 0 1300)))
      (if (> (abs (send *pr2* :head :neck-p :joint-angle)) 30)
	  (ros::ros-error "error head ik ~A ~A" av (v+ #f(0 0 1300) (send tra :worldpos)))
	(send *ri* :head-angle-vector (send *pr2* :angle-vector) 500)))

    (when (and (< 3.0 (send (ros::time- (ros::time-now) *speak-timestamp*) :to-sec))
	       (< 1000 (norm (send tra :worldpos))))
      (setq ang (atan (elt (send tra :worldpos) 1)
		      (elt (send tra :worldpos) 0)))
      (when (< (* 0.25 pi) ang (* 0.50 pi))
	(speak-jp "ひだりに/まがり'ます。")
	(setq *speak-timestamp* (ros::time-now)))
      (when (> (* -0.25 pi) ang (* -0.50 pi))
	(speak-jp "みぎに/まがり'ます。")
	(setq *speak-timestamp* (ros::time-now)))
      (ros::ros-info "angle = ~A" ang)
      )
    (ros::ros-info "~A plan trajectory end point ~A, head angle ~A" *msg* (send tra :worldpos) (send *pr2* :head :angle-vector))
    ))

;; init
(pr2)
(unless (boundp '*ri*) (setq *ri* (instance pr2-interface :init)))
(unless (boundp '*tfl*) (setq *tfl*  (instance ros::transform-listener :init)))
(ros::subscribe "/move_base_node/DWAPlannerROS/global_plan"
		nav_msgs::Path #'global-path-cb 1)
;; look straight when navigation retries
(ros::subscribe "/move_base/result"
		move_base_msgs::MoveBaseActionResult #'result-cb 1)
(ros::advertise "robotsound" sound_play::SoundRequest 5)

(unix::sleep 1)

(ros::spin-once)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))

;;
(ros::rate 5)
(while (ros::ok)
  (ros::sleep)
  (ros::spin-once)
  (look-at-front))

