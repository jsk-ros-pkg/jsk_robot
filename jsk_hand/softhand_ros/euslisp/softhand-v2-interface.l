(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "dynamixel_controllers")
(ros::roseus-add-srvs "dynamixel_controllers")


(defclass softhand-v2-interface
  :super propertied-object
  :slots (finger1-controller-name
          finger2-controller-name
          finger3-controller-name
          finger1-command-topic-name
          finger2-command-topic-name
          finger3-command-topic-name
          finger1-calib-action-client
          finger2-calib-action-client
          finger3-calib-action-client
          finger1-set-torque-limit-service-name
          finger2-set-torque-limit-service-name
          finger3-set-torque-limit-service-name
          heater-controller-name
          heater1-command-topic-name
          heater2-command-topic-name
          heater3-command-topic-name
          thumb-controller-name
          thumb-command-topic-name
  ))

(defmethod softhand-v2-interface
  (:init (&key (calibrate nil) (namespace nil))
    (ros::rate 10)
    (setq finger1-controller-name
          (format nil "~A/finger1_joint_controller" (if namespace namespace "")))
    (setq finger2-controller-name
          (format nil "~A/finger2_joint_controller" (if namespace namespace "")))
    (setq finger3-controller-name
          (format nil "~A/finger3_joint_controller" (if namespace namespace "")))
    (setq heater-controller-name
          (format nil "~A/dxmio_heater_controller" (if namespace namespace "")))
    (setq thumb-controller-name
          (format nil "~A/thumb_joint_controller" (if namespace namespace "")))
    (setq finger1-command-topic-name
          (format nil "~A/command" finger1-controller-name))
    (setq finger2-command-topic-name
          (format nil "~A/command" finger2-controller-name))
    (setq finger3-command-topic-name
          (format nil "~A/command" finger3-controller-name))
    (setq finger1-set-torque-limit-service-name
          (format nil "~A/set_torque_limit" finger1-controller-name))
    (setq finger2-set-torque-limit-service-name
          (format nil "~A/set_torque_limit" finger2-controller-name))
    (setq finger3-set-torque-limit-service-name
          (format nil "~A/set_torque_limit" finger3-controller-name))
    (setq heater1-command-topic-name
          (format nil "~A/command1" heater-controller-name))
    (setq heater2-command-topic-name
          (format nil "~A/command2" heater-controller-name))
    (setq heater3-command-topic-name
          (format nil "~A/command3" heater-controller-name))
    (setq thumb-command-topic-name
          (format nil "~A/command" thumb-controller-name))
    (ros::advertise thumb-command-topic-name std_msgs::Float64 1)
    (ros::advertise finger1-command-topic-name std_msgs::Float64 1)
    (ros::advertise finger2-command-topic-name std_msgs::Float64 1)
    (ros::advertise finger3-command-topic-name std_msgs::Float64 1)
    (ros::advertise heater1-command-topic-name std_msgs::Float64 1)
    (ros::advertise heater2-command-topic-name std_msgs::Float64 1)
    (ros::advertise heater3-command-topic-name std_msgs::Float64 1)
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
    (send self :finger2-rotate rotate-angle)
    (send self :finger3-rotate rotate-angle)
    (send self :finger1-rotate rotate-angle))
  (:stop-grasp (&key (rotate-angle (* pi 2.0)))
    (if (null rotate-angle) (setq rotate-angle (* pi 2.0)))
    (send self :finger1-rotate (/ rotate-angle 2.0))
    (send self :finger2-rotate rotate-angle)
    (send self :finger3-rotate rotate-angle))
  (:open-thumb ()
    (send self :thumb-rotate 0))
  (:close-thumb ()
    (send self :thumb-rotate (* 0.5 pi)))
  (:finger-all-rotate (angle)
    (send self :finger1-rotate angle)
    (send self :finger2-rotate angle)
    (send self :finger3-rotate angle))
  (:thumb-rotate (angle)
    (let ((msg (instance std_msgs::Float64 :init :data angle)))
      (ros::publish thumb-command-topic-name msg)))
  (:finger1-rotate (angle)
    (let ((msg (instance std_msgs::Float64 :init :data angle)))
      (ros::publish finger1-command-topic-name msg)))
  (:finger2-rotate (angle)
    (let ((msg (instance std_msgs::Float64 :init :data angle)))
      (ros::publish finger2-command-topic-name msg)))
  (:finger3-rotate (angle)
    (let ((msg (instance std_msgs::Float64 :init :data angle)))
      (ros::publish finger3-command-topic-name msg)))
  (:start-heater ()
    (send self :set-heater1 0.06)
    (send self :set-heater2 0.06)
    (send self :set-heater3 0.06))
  (:stop-heater ()
    (send self :set-heater1 0.0)
    (send self :set-heater2 0.0)
    (send self :set-heater3 0.0))
  (:set-heater-all (pwm-duty)
    (send self :set-heater1 pwm-duty)
    (send self :set-heater2 pwm-duty)
    (send self :set-heater3 pwm-duty))
  (:set-heater1 (pwm-duty)
    (let ((msg (instance std_msgs::Float64 :init :data pwm-duty)))
      (ros::publish heater1-command-topic-name msg)))
  (:set-heater2 (pwm-duty)
    (let ((msg (instance std_msgs::Float64 :init :data pwm-duty)))
      (ros::publish heater2-command-topic-name msg)))
  (:set-heater3 (pwm-duty)
    (let ((msg (instance std_msgs::Float64 :init :data pwm-duty)))
      (ros::publish heater3-command-topic-name msg)))
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


(defun softhand-v2-init ()
  (ros::roseus "softhand_v2_interface")
  (setq *ri* (instance softhand-v2-interface :init)))
