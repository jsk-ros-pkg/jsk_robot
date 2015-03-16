#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "pr2_mechanism_controllers")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "diagnostic_msgs")

(setq *start-distance* nil)
(setq *start-angle* nil)
(setq *servo-on* nil)

(ros::roseus "active_user_statistics")

(defun motor-cb (msg)
  (ros::ros-debug "motor_state -> ~A" (send msg :data))
  (setq *servo-on* (not (send msg :data))))

(defun odom-cb (msg)
  (ros::ros-debug "odom_state -> distance ~A, angle ~A" (send msg :distance) (send msg :angle))
  (if (or (equal *start-distance* nil) (equal *start-angle* nil))
      (setq *start-distance*  (send msg :distance)
	    *start-angle*     (send msg :angle))
    (setq *distance* (- (send msg :distance) *start-distance*)
          *angle*    (- (send msg :angle)    *start-angle*))))

(defun joint-cb (msg)
  (setq *position* (concatenate float-vector (subseq (send msg :position) 12 16) (subseq (send msg :position) 17)))
  (update-activeness)
  )

(when (ros::has-param "/active_user/motor_subscribe")
  (if (ros::get-param "/active_user/motor_subscribe")
      (ros::subscribe "motor_state" std_msgs::Bool #'motor-cb)
    (setq *servo-on* t)
    )
  )

(when (ros::has-param "/active_user/odom_subscribe")
  (if (ros::get-param "/active_user/odom_subscribe")
      (ros::subscribe "odom_state" pr2_mechanism_controllers::Odometer #'odom-cb)
    )
  )

(when (ros::has-param "/active_user/joint_state_subscribe")
  (if (ros::get-param "/active_user/joint_state_subscribe")
      (ros::subscribe "joint_states" sensor_msgs::JointState  #'joint-cb)
    )
  )

(when (ros::has-param "/active_user/position_diff_threshold")
  (setq *position-diff-threshold* (ros::get-param "/active_user/position_diff_threshold")))



(ros::advertise "diagnostics" diagnostic_msgs::DiagnosticArray 1)

(setq *prev-distance* nil *prev-angle* nil *prev-position* nil)
(setq *distance* nil *angle* nil *position* nil)

(setq *status* 'stop)
(setq *movingp* nil)
(setq *start-time* (ros::time 0))
(setq *elapsed-sec* 0)
(setq *seq* 0)

(when (ros::has-param "/active_user/launch_user_name")
  (let ((tname (ros::get-param "/active_user/launch_user_name")))
    (warn "~%;; launch_user_name = ~A~%" tname)
    (unless (string= tname "false")
      (setq *user-name* tname))))
(unless (and (boundp '*user-name*) *user-name*)
  (unix::system
   (format nil "getent passwd `whoami` | cut -d ':' -f 5 | cut -d ',' -f 1 > /tmp/username_~d.txt"
           (unix::getpid)))
  (warn "Get PID ~A" (unix::getpid))
  (with-open-file
   (f (format nil "/tmp/username_~d.txt" (unix::getpid)))
   (setq *user-name* (read-line f)))
  (ros::set-param "/active_user/launch_user_name" *user-name*)
  )
(warn "~%;; start user_name = ~A~%" *user-name*)

(if (and (ros::has-param "/active_user/elapsed_time")
	 (ros::has-param "/active_user/launch_user_name")
	 (string= (ros::get-param "/active_user/launch_user_name") *user-name*))
    (setq *elapsed* (ros::time (ros::get-param "/active_user/elapsed_time")))
  (setq *elapsed* (ros::time 0)))
(warn "~%;; start elapsed_time with ~A sec~%~%" *elapsed*);;

(let ((tm-now (ros::time-now)))
  (cond
   ((and (ros::has-param "/active_user/start_time")
         (not (ros::time= *elapsed* (ros::time 0))))
    (setq *start-time* (ros::time (ros::get-param "/active_user/start_time"))))
   (t
    (setq *start-time* tm-now)
    (ros::set-param "/active_user/start_time" (send *start-time* :to-sec)))
   ))

(defun update-activeness()
  (ros::ros-debug "user  -> ~A" *user-name*)
  (ros::ros-debug "status-> ~A" *status*)
  (ros::ros-debug "moving-> ~A" *movingp*)
  (setq *odom-disable* (not (ros::get-param "/active_user/odom_subscribe")))
  ;; check if the robot is moving
  (when (and *user-name* (or (and *prev-distance* *prev-angle* *prev-position*) *odom-disable*))
    (let ((diff-distance (if *odom-disable* nil (- *distance* *prev-distance*)))
          (diff-angle (if *odom-disable* nil (- *angle* *prev-angle*)))
          (diff-position (if (not *prev-position*) nil (norm (v- *position* *prev-position*)))))
      ;; check servo on
      (ros::ros-debug " servo on -> ~A" *servo-on*)
      ;; check move_base
      (if (not *odom-disable*)
          (ros::ros-debug " move base -> ~A ~A" diff-distance diff-angle))
      ;; check arms
      (ros::ros-debug " joint-angle  -> ~A" diff-position)
      ;;for odom-enable machine
      (if (and *servo-on*
               (or (> diff-distance 0.001) (> diff-angle 0.001) (> diff-position *position-diff-threshold*))
               (not *odom-disable*))
           (setq *movingp* t)
        ;;for odom-disable machine
        (if (and (and (numberp diff-position) (> diff-position *position-diff-threshold*) )
                 *odom-disable*)
            (setq *movingp* t)
          (setq *movingp* nil))
        )

      ;;
      (cond
       ((and (eq *status* 'stop) *movingp*) ;; stop->move
        (ros::ros-debug "[active_user] start moving")
        (setq *status* 'move)
        (setq *start-time* (ros::time-now))
        (setq *tmp-elapsed* *elapsed*)
        )
       ((and (eq *status* 'stop) (not *movingp*) (not *odom-disable*)) ;; stop->stop
        (setq *tmp-elapsed* (ros::time 0))
        (ros::ros-debug "~A :tmp-elapsed" *tmp-elapsed*)
        )
       ((and (eq *status* 'move) *movingp*) ;; move-> move
        ;; temp elapsed
        (setq *tmp-elapsed* (ros::time+ *elapsed* (ros::time- (ros::time-now) *start-time*)))
        )
       ((and (eq *status* 'move) (not *movingp*)) ;; move-stop
        (ros::ros-debug "[active user] stop moving")
        (setq *status* 'stop)
        (setq *elapsed* (ros::time+ *elapsed* (ros::time- (ros::time-now) *start-time*)))
        )
       )
      (setq *elapsed-sec* (send (if *movingp* *tmp-elapsed* *elapsed*) :to-sec))
      (ros::ros-debug " status  -> ~A (currently moving? ~A), elapsed ~A" *status* *movingp* *elapsed-sec*)
      (ros::publish "diagnostics"
                    (instance diagnostic_msgs::DiagnosticArray :init
                              :header (instance std_msgs::Header :init :seq *seq* :stamp (ros::time-now))
                              :status (list
                                       (instance diagnostic_msgs::DiagnosticStatus :init
                                                 :level 0
                                                 :name"Active User"
                                                 :message (format nil "~A ~A sec" *user-name* *elapsed-sec*)
                                                 :values (list 
                                                          (instance diagnostic_msgs::KeyValue :init
                                                                    :key "User Name"
                                                                    :value *user-name*)
                                                          (instance diagnostic_msgs::KeyValue :init
                                                                    :key "Usage Time"
                                                                    :value *elapsed-sec*))))))
      (incf *seq*)
      ))

  (if (not *odom-disable*)
      (setq *prev-distance* *distance*
            *prev-angle* *angle*
            *prev-position* *position*)
    (setq *prev-position* *position*)
    )
  (when (and (> (length *user-name*) 0) *elapsed-sec*)
    (ros::set-param "/active_user/user_name" *user-name*)
    (ros::set-param "/active_user/elapsed_time" *elapsed-sec*))
  )

(ros::spin)
