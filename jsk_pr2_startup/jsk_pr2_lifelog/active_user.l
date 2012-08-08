#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "pr2_mechanism_controllers")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "diagnostic_msgs")

(setq *start-distance* 0)
(setq *start-angle* 0)

(defun motor-cb (msg)
  (ros::ros-debug "/pr2_etherCAT/motors_halted -> ~A" (send msg :data))
  (setq *servo-on* (not (send msg :data))))

(defun odom-cb (msg)
  (ros::ros-debug "/base_odometry/odometer-> distance ~A, angle ~A" (send msg :distance) (send msg :angle))
  (if (or (= *start-distance* 0) (= *start-angle* 0))
      (setq *start-distance*  (send msg :distance)
	    *start-angle*     (send msg :angle)))
  (setq *distance* (- (send msg :distance) *start-distance*)
	*angle*    (- (send msg :angle)    *start-angle*)))

(defun joint-cb (msg)
  (setq *position* (concatenate float-vector (subseq (send msg :position) 12 16) (subseq (send msg :position) 17)))
  (ros::ros-debug "/joint_states -> ~A" *position*)
  )

(ros::roseus "active_user_statistics")

(ros::subscribe "/pr2_etherCAT/motors_halted" std_msgs::Bool #'motor-cb)
(ros::subscribe "/base_odometry/odometer" pr2_mechanism_controllers::Odometer #'odom-cb)
(ros::subscribe "/joint_states" sensor_msgs::JointState  #'joint-cb)
(ros::advertise "/diagnostics" diagnostic_msgs::DiagnosticArray 1)

(setq *prev-distance* nil *prev-angle* nil *prev-position* nil)
(setq *distance* nil *angle* nil *position* nil)

(ros::rate 1)
(setq *status* 'stop)
(setq *elapsed* (ros::time 0))
(setq *start-time* (ros::time 0))
(setq *elapsed-sec* 0)
(setq *seq* 0)
(do-until-key
  (unix::system (format nil "getent passwd `whoami` | cut -d ':' -f 5 | cut -d ',' -f 1 > /tmp/username_~d.txt" (unix::getpid)))
  (with-open-file (f (format nil "/tmp/username_~d.txt" (unix::getpid)))  (setq *user-name* (read-line f)))
  (ros::ros-info "user -> ~A" *user-name*)
  ;; check if the robot is moving
  (when (and *user-name* *prev-distance* *prev-angle* *prev-position*)
    (let ((diff-distance (- *distance* *prev-distance*))
	  (diff-angle (- *angle* *prev-angle*))
	  (diff-position (norm (v- *position* *prev-position*))))
      ;; check servo on
      (ros::ros-info " servo on -> ~A" *servo-on*)
      ;; check move_base
      (ros::ros-info " move base -> ~A ~A" diff-distance diff-angle)
      ;; check arms
      (ros::ros-info " joint-angle  -> ~A" diff-position)
      (if (and *servo-on*
		 (or (> diff-distance 0.001) (> diff-angle 0.001) (> diff-position 0.001)))
	  (setq *movingp* t)
	(setq *movingp* nil))
      ;;
      (cond
       ((and (eq *status* 'stop) *movingp*) ;; stop->move
	(ros::ros-error "start moving")
	(setq *status* 'move)
	(setq *start-time* (ros::time-now))
	)
       ((and (eq *status* 'stop) (not *movingp*)) ;; stop->stop
	(setq *tmp-elapsed* (ros::time 0))
	)
       ((and (eq *status* 'move) *movingp*) ;; move-> move
	;; temp elapsed
	(setq *tmp-elapsed* (ros::time+ *elapsed* (ros::time- (ros::time-now) *start-time*)))
	)
       ((and (eq *status* 'move) (not *movingp*)) ;; move-stop
	(ros::ros-error "stop moving")
	(setq *status* 'stop)
	(setq *elapsed* (ros::time+ *elapsed* (ros::time- (ros::time-now) *start-time*)))
	))
      (setq *elapsed-sec* (send (if *movingp* *tmp-elapsed* *elapsed*) :to-sec))
      (ros::ros-info " status  -> ~A (currently moving? ~A), elapsed ~A" *status* *movingp* *elapsed-sec*)
      (ros::publish "/diagnostics"
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
  (setq *prev-distance* *distance*
	*prev-angle* *angle*
	*prev-position* *position*)
  (ros::spin-once)
  (when (and *user-name* *elapsed-sec*)
    (ros::set-param "/active_user/user_name" *user-name*)
    (ros::set-param "/active_user/elapsed_time" *elapsed-sec*))
  (ros::sleep)
  )
