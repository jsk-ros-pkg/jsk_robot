#!/usr/bin/env roseus

(require "package://panda_eus/euslisp/dual_panda-interface.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "pose-stamped-publisher")

(setq *robot* (dual_panda))

(ros::advertise "/dual_panda/end_coords/rarm" geometry_msgs::PoseStamped 1)
(ros::advertise "/dual_panda/end_coords/larm" geometry_msgs::PoseStamped 1)

(defvar *joints* nil)

(defun joints-cb (msg)
  (setq buf (send msg :position))
  (setq buf (coerce buf cons))
  (setq l (- (length buf) 1))
  (setq i l)
  (setq ans nil)
  (print "publishing end-coords...")
  (while (<= 0 i)
    (if (and (not (= i 7)) (not (= i 8)) (not (= i 16)) (not (= i 17)))
	(if (not ans)
	    (setq ans (cons (elt buf i) nil))
	  (setq ans (cons (elt buf i) ans))))
    (setq i (- i 1)))
  (setq *joints* (coerce ans float-vector)))

(ros::rate 10)
(ros::subscribe "/dual_panda/joint_states" sensor_msgs::JointState #'joints-cb 1)

(while (ros::ok)
  (let ((pose-stamped-rarm (instance geometry_msgs::PoseStamped :init) (pose-stamped-larm (instance geometry_msgs::PoseStamped :init))))
    (setq joint-states (coerce (mapcar #'(lambda (x) (rad2deg x)) (coerce *joints* cons)) float-vector))
    (if (= (length joint-states) 14)
	(send *robot* :angle-vector joint-states))
    (setq pose-stamped-rarm (ros::coords->tf-pose-stamped (send *robot* :rarm :end-coords) "dual_arm_base"))
    (setq pose-stamped-larm (ros::coords->tf-pose-stamped (send *robot* :larm :end-coords) "dual_arm_base"))
    (ros::publish "/dual_panda/end_coords/rarm" pose-stamped-rarm)
    (ros::publish "/dual_panda/end_coords/larm" pose-stamped-larm)
    (ros::spin-once)
    (ros::sleep)))
