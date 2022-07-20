(require :dynamixel-hand-interface "package://softhand_ros/euslisp/dynamixel-hand-interface.l")


(defclass softhand-interface
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
  ))

(defmethod softhand-interface
  (:init (&key (calibrate nil) (namespace nil))
    (send-super :init :finger-num 3 :namespace namespace)
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
    (send self :finger-all-rotate rotate-angle))
  (:stop-grasp (&key (rotate-angle (* pi 1.5)))
    (if (null rotate-angle) (setq rotate-angle (* pi 1.5)))
    (send self :finger-all-rotate rotate-angle)))


(defun softhand-init ()
  (ros::roseus "softhand_interface")
  (setq *ri* (instance softhand-interface :init)))
