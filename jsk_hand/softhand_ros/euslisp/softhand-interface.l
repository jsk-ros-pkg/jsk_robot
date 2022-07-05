(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "dynamixel_msgs")
(ros::roseus-add-msgs "dynamixel_controllers")
(ros::roseus-add-srvs "dynamixel_controllers")


(defclass softhand-interface
  :super propertied-object
  :slots (finger1-controller-name
          finger2-controller-name
          finger3-controller-name
          finger1-command-topic-name
          finger2-command-topic-name
          finger3-command-topic-name
          finger1-state-topic-name
          finger2-state-topic-name
          finger3-state-topic-name
          finger1-calib-action-client
          finger2-calib-action-client
          finger3-calib-action-client
          finger1-set-torque-limit-service-name
          finger2-set-torque-limit-service-name
          finger3-set-torque-limit-service-name
          finger1-state-msg
          finger2-state-msg
          finger3-state-msg
  ))

(defmethod softhand-interface
  (:init (&key (calibrate nil) (namespace nil))
    (ros::rate 10)
    (setq finger1-controller-name
          (format nil "~A/finger1_joint_controller" (if namespace namespace "")))
    (setq finger2-controller-name
          (format nil "~A/finger2_joint_controller" (if namespace namespace "")))
    (setq finger3-controller-name
          (format nil "~A/finger3_joint_controller" (if namespace namespace "")))
    (setq finger1-command-topic-name
          (format nil "~A/command" finger1-controller-name))
    (setq finger2-command-topic-name
          (format nil "~A/command" finger2-controller-name))
    (setq finger3-command-topic-name
          (format nil "~A/command" finger3-controller-name))
    (setq finger1-state-topic-name
          (format nil "~A/state" finger1-controller-name))
    (setq finger2-state-topic-name
          (format nil "~A/state" finger2-controller-name))
    (setq finger3-state-topic-name
          (format nil "~A/state" finger3-controller-name))
    (setq finger1-set-torque-limit-service-name
          (format nil "~A/set_torque_limit" finger1-controller-name))
    (setq finger2-set-torque-limit-service-name
          (format nil "~A/set_torque_limit" finger2-controller-name))
    (setq finger3-set-torque-limit-service-name
          (format nil "~A/set_torque_limit" finger3-controller-name))
    (ros::advertise finger1-command-topic-name std_msgs::Float64 1)
    (ros::advertise finger2-command-topic-name std_msgs::Float64 1)
    (ros::advertise finger3-command-topic-name std_msgs::Float64 1)
    (ros::subscribe finger1-state-topic-name
                    dynamixel_msgs::JointState #'send self :finger1-state-cb)
    (ros::subscribe finger2-state-topic-name
                    dynamixel_msgs::JointState #'send self :finger2-state-cb)
    (ros::subscribe finger3-state-topic-name
                    dynamixel_msgs::JointState #'send self :finger3-state-cb)
    (setq finger1-calib-action-client
          (instance ros::simple-action-client :init
                    (format nil "~A/calib" finger1-controller-name)
                    dynamixel_controllers::CalibJointAction))
    (setq finger2-calib-action-client
          (instance ros::simple-action-client :init
                    (format nil "~A/calib" finger2-controller-name)
                    dynamixel_controllers::CalibJointAction))
    (setq finger3-calib-action-client
          (instance ros::simple-action-client :init
                    (format nil "~A/calib" finger3-controller-name)
                    dynamixel_controllers::CalibJointAction))
    (when calibrate (send self :calibrate))
    self)
  (:calibrate ()
    (send self :calibrate-finger1)
    (send self :calibrate-finger2)
    (send self :calibrate-finger3))
  (:calibrate-finger1 ()
    (send self :start-finger1-calib)
    (send (instance ros::Duration :init 3) :sleep)
    (while (send self :finger1-calibratingp)
      (ros::spin-once)))
  (:calibrate-finger2 ()
    (send self :start-finger2-calib)
    (send (instance ros::Duration :init 3) :sleep)
    (while (send self :finger2-calibratingp)
      (ros::spin-once)))
  (:calibrate-finger3 ()
    (send self :start-finger3-calib)
    (send (instance ros::Duration :init 3) :sleep)
    (while (send self :finger3-calibratingp)
      (ros::spin-once)))
  (:start-finger1-calib ()
    (send finger1-calib-action-client :send-goal
          (instance dynamixel_controllers::CalibJointActionGoal :init)))
  (:start-finger2-calib ()
    (send finger2-calib-action-client :send-goal
          (instance dynamixel_controllers::CalibJointActionGoal :init)))
  (:start-finger3-calib ()
    (send finger3-calib-action-client :send-goal
          (instance dynamixel_controllers::CalibJointActionGoal :init)))
  (:finger1-calibratingp ()
    (ros::spin-once)
    (eq (send finger1-calib-action-client :get-state) ros::*simple-goal-state-active*))
  (:finger2-calibratingp ()
    (ros::spin-once)
    (eq (send finger2-calib-action-client :get-state) ros::*simple-goal-state-active*))
  (:finger3-calibratingp ()
    (ros::spin-once)
    (eq (send finger3-calib-action-client :get-state) ros::*simple-goal-state-active*))
  (:start-grasp (&key (rotate-angle (* pi -2.0)))
    (if (null rotate-angle) (setq rotate-angle (* pi -2.0)))
    (send self :finger-all-rotate rotate-angle))
  (:stop-grasp (&key (rotate-angle (* pi 1.5)))
    (if (null rotate-angle) (setq rotate-angle (* pi 1.5)))
    (send self :finger-all-rotate rotate-angle))
  (:finger-all-rotate (angle)
    (send self :finger1-rotate angle)
    (send self :finger2-rotate angle)
    (send self :finger3-rotate angle))
  (:finger1-rotate (angle)
    (let ((msg (instance std_msgs::Float64 :init :data angle)))
      (ros::publish finger1-command-topic-name msg)))
  (:finger2-rotate (angle)
    (let ((msg (instance std_msgs::Float64 :init :data angle)))
      (ros::publish finger2-command-topic-name msg)))
  (:finger3-rotate (angle)
    (let ((msg (instance std_msgs::Float64 :init :data angle)))
      (ros::publish finger3-command-topic-name msg)))
  (:finger1-state-cb (msg)
    (setq finger1-state-msg msg)
    ;;(print (list 'cb self msg))
    )
  (:finger2-state-cb (msg)
    (setq finger2-state-msg msg)
    ;; (print (list 'cb self msg))
    )
  (:finger3-state-cb (msg)
    (setq finger3-state-msg msg)
    ;; (print (list 'cb self msg))
    )
  (:get-finger1-state-msg (&key (timeout 10))
    (let ((now (ros::time-now)))
      (while (or (not finger1-state-msg)
    (not (> (send (send finger1-state-msg :header :stamp) :to-nsec)
        (send now :to-nsec))))
    (progn
          ;; (ros::ros-warn "waiting for new topic arrival")
          (ros::spin-once)
      (if (> (- (send (ros::time-now) :to-sec)
            (send now :to-sec)) timeout)
          (return-from :get-finger1-state-msg nil))))
      (return-from :get-finger1-state-msg finger1-state-msg)
      ))
  (:get-finger2-state-msg (&key (timeout 10))
    (let ((now (ros::time-now)))
      (while (or (not finger2-state-msg)
    (not (> (send (send finger2-state-msg :header :stamp) :to-nsec)
        (send now :to-nsec))))
    (progn
          ;; (ros::ros-warn "waiting for new topic arrival")
          (ros::spin-once)
      (if (> (- (send (ros::time-now) :to-sec)
            (send now :to-sec)) timeout)
          (return-from :get-finger2-state-msg nil))))
      (return-from :get-finger2-state-msg finger2-state-msg)
      ))
  (:get-finger3-state-msg (&key (timeout 10))
    (let ((now (ros::time-now)))
      (while (or (not finger3-state-msg)
    (not (> (send (send finger3-state-msg :header :stamp) :to-nsec)
        (send now :to-nsec))))
    (progn
          ;; (ros::ros-warn "waiting for new topic arrival")
          (ros::spin-once)
      (if (> (- (send (ros::time-now) :to-sec)
            (send now :to-sec)) timeout)
          (return-from :get-finger3-state-msg nil))))
      (return-from :get-finger3-state-msg finger3-state-msg)
      ))
  (:get-finger1-current-pos ()
    (let ((finger-state-msg))
      (setq finger-state-msg (send self :get-finger1-state-msg))
      (if finger-state-msg
    (progn
      (setq finger-current-pos (send finger-state-msg :current_pos))
      (return-from :get-finger1-current-pos finger-current-pos))
    (return-from :get-finger1-current-pos nil))))
  (:get-finger2-current-pos ()
    (let ((finger-state-msg))
      (setq finger-state-msg (send self :get-finger2-state-msg))
      (if finger-state-msg
    (progn
      (setq finger-current-pos (send finger-state-msg :current_pos))
      (return-from :get-finger2-current-pos finger-current-pos))
    (return-from :get-finger2-current-pos nil))))
  (:get-finger3-current-pos ()
    (let ((finger-state-msg))
      (setq finger-state-msg (send self :get-finger3-state-msg))
      (if finger-state-msg
    (progn
      (setq finger-current-pos (send finger-state-msg :current_pos))
      (return-from :get-finger3-current-pos finger-current-pos))
    (return-from :get-finger3-current-pos nil))))
  (:set-torque-limit-all (torque)
    (send self :set-finger1-torque-limit torque)
    (send self :set-finger2-torque-limit torque)
    (send self :set-finger3-torque-limit torque))
  (:set-finger1-torque-limit (torque)
    (let ((req (instance dynamixel_controllers::SetTorqueLimitRequest :init
                         :torque_limit torque)))
      (ros::service-call finger1-set-torque-limit-service-name req)))
  (:set-finger2-torque-limit (torque)
    (let ((req (instance dynamixel_controllers::SetTorqueLimitRequest :init
                         :torque_limit torque)))
      (ros::service-call finger2-set-torque-limit-service-name req)))
  (:set-finger3-torque-limit (torque)
    (let ((req (instance dynamixel_controllers::SetTorqueLimitRequest :init
                         :torque_limit torque)))
      (ros::service-call finger3-set-torque-limit-service-name req))))


(defun softhand-init ()
  (ros::roseus "softhand_interface")
  (setq *ri* (instance softhand-interface :init)))
