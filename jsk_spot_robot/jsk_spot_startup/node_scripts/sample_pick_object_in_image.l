#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "spot_msgs")

(ros::roseus "sample_pick_object_in_image")
(setq *c* (instance ros::simple-action-client :init
		    "/spot/pick_object_in_image" spot_msgs::PickObjectInImageAction))
(send *c* :wait-for-server)

;; call-trigger-service reqrueis roseus>1.7.5
;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/705
;; 
(defun ros::call-trigger-service (srvname &key (wait nil) (timeout -1) (persistent nil))
  "Call std_srv/Trigger service. Use (setq r (call-trigger-service \"/test\" t)) (and r (send r :success)) to check if it succeed. If r is nil, it fail to find service."
  (let (r)
    (when (ros::wait-for-service srvname (if wait timeout 0))
      (setq r (ros::service-call srvname (instance std_srvs::TriggerRequest :init) persistent))
      (ros::ros-debug "Call \"~A\" returns \"~A\"" srvname (send r :message))
      (unless (send r :success) (ros::ros-warn "Call \"~A\" fails, it returns \"~A\"" srvname (send r :message)))
      r)))

(ros::subscribe "screenpoint" geometry_msgs::PointStamped
		#'(lambda (msg)
		    (let ((goal (instance spot_msgs::PickObjectInImageActionGoal :init)))
		      (send goal :goal :image_source (ros::get-param "~image_source")) ;; required ex) "hand_color_image"
		      (send goal :goal :center (instance geometry_msgs::Point :init :x (send msg :point :x) :y (send msg :point :y)))
		      (send goal :goal :max_duration (instance ros::duration :init 15))
		      (send *c* :send-goal goal))))

(do-until-key
 (ros::rate 3)
 (while (memq (send (send (*c* . ros::comm-state) :latest-goal-status) :status) ;; bugs in actionlib.l??
	      (list actionlib_msgs::GoalStatus::*pending*
		    actionlib_msgs::GoalStatus::*preempted*
		    actionlib_msgs::GoalStatus::*succeeded*
		    actionlib_msgs::GoalStatus::*aborted*))
   (ros::ros-warn "waiting to select target object in image_view ...")
   (ros::sleep)
   (ros::spin-once))
 (ros::ros-warn ";; wait-for-result")
 (send *c* :wait-for-result)
 (if (send (send *c* :get-result) :success)
     ;; if robot grab object, unstow then open gripper
     (progn
       (ros::call-trigger-service "/spot/unstow_arm")
       (ros::call-trigger-service "/spot/gripper_open"))
   (progn
     ;; if robot failed to grab object, just open gripper
     (ros::call-trigger-service "/spot/gripper_open")))
 (ros::call-trigger-service "/spot/stow_arm")
 )





