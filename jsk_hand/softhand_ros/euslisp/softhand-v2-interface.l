(require :dynamixel-hand-interface "package://softhand_ros/euslisp/dynamixel-hand-interface.l")


(defclass softhand-v2-interface
  :super dynamixel-hand-interface
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
    (send-super :init :finger-num 3 :namespace namespace)
    (setq heater-controller-name
          (format nil "~A/dxmio_heater_controller" (if namespace namespace "")))
    (dotimes (i 3)
      (let* ((hid (+ i 1))
             (hctname (format nil "~A/command~A" heater-controller-name hid)))
        (send self :set-val (read-from-string (format nil "heater~A-command-topic-name" hid)) hctname)
        (ros::advertise hctname std_msgs::Float64 1)))
    (setq thumb-controller-name
          (format nil "~A/thumb_joint_controller" (if namespace namespace "")))
    (setq thumb-command-topic-name
          (format nil "~A/command" thumb-controller-name))
    (ros::advertise thumb-command-topic-name std_msgs::Float64 1)
    (when calibrate (send self :calibrate))
    self)
  (:calibrate-finger1 () (send self :calibrate-finger-impl 1))
  (:calibrate-finger2 () (send self :calibrate-finger-impl 2))
  (:calibrate-finger3 () (send self :calibrate-finger-impl 3))
  (:start-finger1-calib () (send self :start-finger-calib-impl 1))
  (:start-finger2-calib () (send self :start-finger-calib-impl 2))
  (:start-finger3-calib () (send self :start-finger-calib-impl 3))
  (:finger1-calibratingp () (send self :finger-calibratingp-impl 1))
  (:finger2-calibratingp () (send self :finger-calibratingp-impl 2))
  (:finger3-calibratingp () (send self :finger-calibratingp-impl 3))
  (:finger1-rotate (angle) (send self :finger-rotate-impl 1 angle))
  (:finger2-rotate (angle) (send self :finger-rotate-impl 2 angle))
  (:finger3-rotate (angle) (send self :finger-rotate-impl 3 angle))
  (:get-finger1-current-pos () (send self :get-finger-current-pos-impl 1))
  (:get-finger2-current-pos () (send self :get-finger-current-pos-impl 2))
  (:get-finger3-current-pos () (send self :get-finger-current-pos-impl 3))
  (:set-finger1-torque-limit (torque) (send self :set-finger-torque-limit-impl 1 torque))
  (:set-finger2-torque-limit (torque) (send self :set-finger-torque-limit-impl 2 torque))
  (:set-finger3-torque-limit (torque) (send self :set-finger-torque-limit-impl 3 torque))
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
  ;; v2 specific methods
  ;; thumb methods
  (:thumb-rotate (angle)
    (let ((msg (instance std_msgs::Float64 :init :data angle)))
      (ros::publish thumb-command-topic-name msg)))
  (:open-thumb ()
    (send self :thumb-rotate 0))
  (:close-thumb ()
    (send self :thumb-rotate (* 0.5 pi)))
  ;; heater methods
  (:set-heater-impl (hid pwm-duty)
    (let ((msg (instance std_msgs::Float64 :init :data pwm-duty))
          (hctname (eval (read-from-string (format nil "heater~A-command-topic-name" hid)))))
      (ros::publish hctname msg)))
  (:set-heater-all (pwm-duty)
    (send self :set-heater-impl 1 pwm-duty)
    (send self :set-heater-impl 2 pwm-duty)
    (send self :set-heater-impl 3 pwm-duty))
  (:set-heater1 (pwm-duty) (send self :set-heater-impl 1 pwm-duty))
  (:set-heater2 (pwm-duty) (send self :set-heater-impl 2 pwm-duty))
  (:set-heater3 (pwm-duty) (send self :set-heater-impl 3 pwm-duty))
  (:start-heater () (send self :set-heater-all 0.06))
  (:stop-heater () (send self :set-heater-all 0.0)))


(defun softhand-v2-init ()
  (ros::roseus "softhand_v2_interface")
  (setq *ri* (instance softhand-v2-interface :init)))
