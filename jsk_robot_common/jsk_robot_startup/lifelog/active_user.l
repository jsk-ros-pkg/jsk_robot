#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "pr2_mechanism_controllers")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "diagnostic_msgs")

(defparameter *init-status* '(nil))
(defparameter *prev-status* '(nil))
(defparameter *current-status* '(nil))
(defparameter *diagnostic-seq* 0)
(defparameter *position-diff-threshold* 0.05)

(defun set-alist (k v alist &key (key 'car) (test 'eq))
  (let ((cur-cons (assoc k alist :key key :test test)))
    (if cur-cons
        (progn (setf (cdr cur-cons) v) alist)
        (nconc alist (list (cons k v))))))

(defun status-cb (key msg)
  (unless (assoc key *init-status*)
    (set-alist key msg *init-status*))
  (when (assoc key *current-status*)
    (set-alist key (cdr (assoc key *current-status*)) *prev-status*))
  (set-alist key msg *current-status*))

(defun init-status ()
  (setq *position-diff-threshold* (ros::get-param "/active_user/position_diff_threshold" 0.05))
  (setq *init-status* '(nil))
  ;; user name
  (let ((name (ros::get-param "/active_user/launch_user_name" nil)))
    (when (or (null name) (null-string-p name) (string= name "false"))
      (unix::system
       (format nil "getent passwd `whoami` | cut -d ':' -f 5 | cut -d ',' -f 1 > /tmp/username_~d.txt"
               (unix::getpid)))
      (with-open-file
          (f (format nil "/tmp/username_~d.txt" (unix::getpid)))
        (setq name (read-line f)))
      (when (or (null name) (null-string-p name))
        (setq name *user*)))
    (set-alist :user-name name *init-status*))
  ;; robot name
  (let ((name (ros::get-param "/robot/type" nil)))
    (when (or (null name) (string= name "false"))
      (if (ros::has-param "/rethink/software_version")
          (setq name "baxter")))
    (set-alist :robot-name name *init-status*))
  ;; elapsed time
  (set-alist :elapsed-time
             (ros::time (ros::get-param "/active_user/elapsed_time" 0))
             *init-status*)
  ;; start time
  (let ((cur-time (send (ros::time-now) :to-sec)))
    (set-alist :start-time
               (ros::time (ros::get-param "/active_user/start_time" cur-time))
               *init-status*)
    (ros::set-param "/active_user/start_time" cur-time))
  )

(defun joint-position (robot msg)
  (setq robot (string-downcase robot))
  (when msg
    (cond
      ((string= robot "pr2")
       (concatenate float-vector
                    (subseq (send msg :position) 12 16)
                    (subseq (send msg :position) 17)))
      ((string= robot "baxter")
       (if (< (length (send msg :position)) 12)
           nil (send msg :position)))
      (t (send msg :position)))))

(defun publish-diagnostics (username elapsed)
  (when (derivedp elapsed ros::time)
    (setq elapsed (format nil "~A" (send elapsed :to-sec))))
  (let (kvs)
    (when username
      (push (instance diagnostic_msgs::KeyValue :init
                      :key "User Name"
                      :value username) kvs))
    (when elapsed
      (push (instance diagnostic_msgs::KeyValue :init
                      :key "Usage Time"
                      :value elapsed) kvs))
    (ros::publish "diagnostics"
                  (instance diagnostic_msgs::DiagnosticArray :init
                            :header (instance std_msgs::Header :init :seq (incf *diagnostic-seq*) :stamp (ros::time-now))
                            :status (list
                                     (instance diagnostic_msgs::DiagnosticStatus :init
                                               :level 0
                                               :name"Active User"
                                               :message (format nil "~A ~A sec" username elapsed)
                                               :values kvs))))))

(defun update ()
  (let ((movingp nil) (servo-on nil))
    (ros::ros-debug "current: ~A" *current-status*)
    ;; motor
    (cond
      ((assoc :motor *current-status*)
       (setq servo-on (send (cdr (assoc :motor *current-status*)) :data)))
      (t (setq servo-on t)))
    ;;
    (when servo-on
      ;; odom
      (let ((odom (cdr (assoc :odom *current-status*)))
            (prev-odom (cdr (assoc :odom *prev-status*)))
            (diff-distance 0.0)
            (diff-angle 0.0))
        (when (and odom prev-odom)
          (setq diff-distance (apply #'- (send-all (list odom prev-odom) :distance)))
          (setq diff-angle (apply #'- (send-all (list odom prev-odom) :angle)))
          (when (or (> diff-distance 0.001) (> diff-angle 0.001))
            (setq movingp t))))
      ;; joint
      (unless movingp
        (let* ((robot-name (cdr (assoc :robot-name *init-status*)))
               (pos (joint-position robot-name (cdr (assoc :joint *current-status*))))
               (prev-pos (joint-position robot-name (cdr (assoc :joint *prev-status*))))
               (diff-position 0.0))
          (when (and pos prev-pos)
            (setq diff-position (norm (v- pos prev-pos)))
            (when (> diff-position *position-diff-threshold*)
              (setq movingp t))))))
    ;;
    (let ((prev-movingp (cdr (assoc :movingp *prev-status*)))
          (start-time (or (cdr (assoc :start-time *current-status*))
                          (cdr (assoc :start-time *init-status*))))
          (elapsed (or (cdr (assoc :elapsed-time *current-status*))
                       (cdr (assoc :elapsed-time *init-status*)))))
      (cond
        ((and (not prev-movingp) movingp)
         ;; stop -> move
         (setq start-time (ros::time-now)))
        ((and prev-movingp (not movingp))
         ;; move -> stop
         (setq elapsed (ros::time+ elapsed (ros::time- (ros::time-now) start-time)))))
      ;;
      (set-alist :start-time start-time *current-status*)
      (set-alist :elapsed-time elapsed *current-status*)
      ;;
      (publish-diagnostics (cdr (assoc :user-name *init-status*)) elapsed)
      (ros::set-param "/active_user/elapsed_time" (send elapsed :to-sec)))

    (set-alist :movingp (cdr (assoc :movingp *current-status*)) *prev-status*)
    (set-alist :movingp movingp *current-status*)
    ))

(defun main ()
  (ros::roseus "active_user_statistics")
  ;;
  (init-status)
  ;;
  (ros::advertise "diagnostics" diagnostic_msgs::DiagnosticArray 1)
  ;;
  (when (ros::get-param "/active_user/motor_subscribe" t)
    (ros::subscribe "motor_state" std_msgs::Bool #'status-cb :motor 1))
  (when (ros::get-param "/active_user/odom_subscribe" t)
    (ros::subscribe "odom_state" pr2_mechanism_controllers::Odometer #'status-cb :odom 1))
  (when (ros::get-param "/active_user/joint_state_subscribe" t)
    (ros::subscribe "joint_states" sensor_msgs::JointState  #'status-cb :joint 1))

  (ros::rate 0.1)
  (while (ros::ok)
    (ros::spin-once)
    (update)
    (ros::sleep)))

(main)
(exit)
