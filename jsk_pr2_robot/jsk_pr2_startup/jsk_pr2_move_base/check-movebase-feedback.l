#!/usr/bin/env roseus

(ros::load-ros-manifest "pr2eus")
(ros::roseus "move_base_simple")

(setq *last-message* (ros::time-now))
(defun feedback-cb (msg)
  (setq *last-message* (ros::time-now)))

(setq *goal* nil)
(defun goal-cb (msg)
  (setq *goal* (send msg :goal :target_pose)))

(setq *status* nil)
(defun status-cb (msg)
  (setq *status* (= (send (elt (send msg :status_list) 0) :status) 1))
  (unless *status* (setq *last-message* (ros::time-now)))
  )

(ros::subscribe "/move_base/feedback" move_base_msgs::MoveBaseActionFeedback #'feedback-cb)
(ros::subscribe "/move_base/goal" move_base_msgs::MoveBaseActionGoal #'goal-cb)
(ros::subscribe "/move_base/status" actionlib_msgs::GoalStatusArray #'status-cb)
(ros::advertise "/move_base_simple/goal" geometry_msgs::PoseStamped)

(ros::rate 2)
(do-until-key
 (ros::sleep)
 (ros::spin-once)
 (setq interval (send (ros::time- (ros::time-now) *last-message*) :to-sec))
 (ros::ros-info "check /move_base/status ~A, /move_base/feedback interval ~5,2A[sec], move_base/goal ~A" *status* interval (if *goal* (ros::tf-pose->coords (send *goal* :pose))))
 (when (and (> interval 5) *goal* *status*)
   ;; kick move_base_simple
   (when (y-or-n-p "Send Simple Goal??")
	 (ros::ros-warn "publish /move_base_simple/goal ~A" *goal*)
	 (ros::publish "/move_base_simple/goal" *goal*))
   (setq *last-message* (ros::time-now)))
 )




