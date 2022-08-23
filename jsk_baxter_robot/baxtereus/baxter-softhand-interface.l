(load "package://baxtereus/baxter-interface.l")
(load "package://softhand_ros/euslisp/softhand-interface.l")
(load "package://softhand_ros/euslisp/softhand-v2-interface.l")


(defclass baxter-softhand-interface
  :super baxter-interface
  :slots (lgripper-type
          lgripper-interface
          rgripper-type
          rgripper-interface))


(defmethod baxter-softhand-interface
  (:init (&rest args &key (lgripper :parallel) (rgripper :parallel) &allow-other-keys)
    (send-super* :init args)
    (setq lgripper-type lgripper)
    (setq rgripper-type rgripper)
    (setq lgripper-interface
      (cond
        ((equal lgripper-type :softhand)
         (instance softhand-interface :init :namespace "/lgripper"))
        ((equal lgripper-type :softhand-v2)
         (instance softhand-v2-interface :init :namespace "/lgripper"))
        (t nil)))
    (setq rgripper-interface
      (cond
        ((equal rgripper-type :softhand)
         (instance softhand-interface :init :namespace "/rgripper"))
        ((equal rgripper-type :softhand-v2)
         (instance softhand-v2-interface :init :namespace "/rgripper"))
        (t nil))))
  (:get-gripper-type (arm)
    (cond ((equal arm :larm) lgripper-type)
          ((equal arm :rarm) rgripper-type)
          ((equal arm :arms)
           (list lgripper-type rgripper-type))
          (t nil)))
  (:get-gripper-interface (arm)
    (cond ((equal arm :larm) lgripper-interface)
          ((equal arm :rarm) rgripper-interface)
          ((equal arm :arms)
           (list lgripper-interface rgripper-interface))
          (t nil)))
  (:start-grasp (&optional (arm :arms) &key (rotate-angle nil) &rest args &allow-other-keys)
    (cond
      ((equal arm :arms)
       (let ((larm-start-grasp-p (send* self :start-grasp-step :larm :rotate-angle rotate-angle args))
             (rarm-start-grasp-p (send* self :start-grasp-step :rarm :rotate-angle rotate-angle args)))
         (and larm-start-grasp-p rarm-start-grasp-p)))
      (t (send* self :start-grasp-step arm :rotate-angle rotate-angle args))))
  (:start-grasp-step (arm &key (rotate-angle nil) &rest args &allow-other-keys)
    (cond
      ((equal (send self :get-gripper-type arm) :parallel)
       (send-super* :start-grasp arm args))
      ((or (equal (send self :get-gripper-type arm) :softhand)
           (equal (send self :get-gripper-type arm) :softhand-v2))
       (send (send self :get-gripper-interface arm) :start-grasp :rotate-angle rotate-angle))
      (t nil)))
  (:stop-grasp (&optional (arm :arms) &key (rotate-angle nil) &rest args &allow-other-keys)
    (cond
      ((equal arm :arms)
       (let ((larm-stop-grasp-p (send* self :stop-grasp-step :larm :rotate-angle rotate-angle args))
             (rarm-stop-grasp-p (send* self :stop-grasp-step :rarm :rotate-angle rotate-angle args)))
         (and larm-stop-grasp-p rarm-stop-grasp-p)))
      (t (send* self :stop-grasp-step arm args))))
  (:stop-grasp-step (arm &key (rotate-angle nil) &rest args &allow-other-keys)
    (cond
      ((equal (send self :get-gripper-type arm) :parallel)
       (send-super* :stop-grasp arm args))
      ((or (equal (send self :get-gripper-type arm) :softhand)
           (equal (send self :get-gripper-type arm) :softhand-v2))
       (send (send self :get-gripper-interface arm) :stop-grasp :rotate-angle rotate-angle))
      (t nil)))
  (:finger1-rotate (arm rotate-angle) (send self :finger-rotate arm 1 rotate-angle))
  (:finger2-rotate (arm rotate-angle) (send self :finger-rotate arm 2 rotate-angle))
  (:finger3-rotate (arm rotate-angle) (send self :finger-rotate arm 3 rotate-angle))
  (:finger-rotate (arm fid rotate-angle)
    (cond
      ((equal arm :arms)
       (let ((larm-finger-rotate-p (send self :finger-rotate-step :larm fid rotate-angle))
             (rarm-finger-rotate-p (send self :finger-rotate-step :rarm fid rotate-angle)))
         (and larm-finger-rotate-p rarm-finger-rotate-p)))
      (t (send self :finger-rotate-step arm fid rotate-angle))))
  (:finger-rotate-step (arm fid rotate-angle)
    (cond
      ((equal (send self :get-gripper-type arm) :parallel) nil)
      ((or (equal (send self :get-gripper-type arm) :softhand)
           (equal (send self :get-gripper-type arm) :softhand-v2))
       (send (send self :get-gripper-interface arm) :finger-rotate-impl fid rotate-angle))
      (t nil)))
  (:open-thumb (&optional (arm :arms))
    (cond
      ((equal arm :arms)
       (let ((larm-open-thumb-p (send self :open-thumb-step :larm))
             (rarm-open-thumb-p (send self :open-thumb-step :rarm)))
         (and larm-open-thumb-p rarm-open-thumb-p)))
      (t (send self :open-thumb-step arm))))
  (:open-thumb-step (arm)
    (cond
      ((equal (send self :get-gripper-type arm) :softhand-v2)
       (send (send self :get-gripper-interface arm) :open-thumb))
      (t
       (ros::ros-warn ":open-thumb is invalid for this gripper type")
       nil)))
  (:close-thumb (&optional (arm :arms))
    (cond
      ((equal arm :arms)
       (let ((larm-close-thumb-p (send self :close-thumb-step :larm))
             (rarm-close-thumb-p (send self :close-thumb-step :rarm)))
         (and larm-close-thumb-p rarm-close-thumb-p)))
      (t (send self :close-thumb-step arm))))
  (:close-thumb-step (arm)
    (cond
      ((equal (send self :get-gripper-type arm) :softhand-v2)
       (send (send self :get-gripper-interface arm) :close-thumb))
      (t
       (ros::ros-warn ":close-thumb is invalid for this gripper type")
       nil)))
  (:start-heater (&optional (arm :arms))
    (cond
      ((equal arm :arms)
       (let ((larm-start-heater-p (send self :start-heater-step :larm))
             (rarm-start-heater-p (send self :start-heater-step :rarm)))
         (and larm-start-heater-p rarm-start-heater-p)))
      (t (send self :start-heater-step arm))))
  (:start-heater-step (arm)
    (cond
      ((equal (send self :get-gripper-type arm) :softhand-v2)
       (send (send self :get-gripper-interface arm) :start-heater))
      (t
       (ros::ros-warn ":start-heater is invalid for this gripper type")
       nil)))
  (:stop-heater (&optional (arm :arms))
    (cond
      ((equal arm :arms)
       (let ((larm-stop-heater-p (send self :stop-heater-step :larm))
             (rarm-stop-heater-p (send self :stop-heater-step :rarm)))
         (and larm-stop-heater-p rarm-stop-heater-p)))
      (t (send self :stop-heater-step arm))))
  (:stop-heater-step (arm)
    (cond
      ((equal (send self :get-gripper-type arm) :softhand-v2)
       (send (send self :get-gripper-interface arm) :stop-heater))
      (t
       (ros::ros-warn ":stop-heater is invalid for this gripper type")
       nil)))
  (:thumb-rotate (arm angle)
    (cond
      ((equal arm :arms)
       (let ((larm-thumb-rotate-p (send self :thumb-rotate-step :larm angle))
             (rarm-thumb-rotate-p (send self :thumb-rotate-step :rarm angle)))
         (and larm-thumb-rotate-p rarm-thumb-rotate-p)))
      (t (send self :thumb-rotate-step arm angle))))
  (:thumb-rotate-step (arm angle)
    (cond
      ((equal (send self :get-gripper-type arm) :softhand-v2)
       (send (send self :get-gripper-interface arm) :thumb-rotate angle))
      (t
       (ros::ros-warn ":thumb-rotate is invalid for this gripper type")
       nil)))
  (:set-torque-limit (arm torque)
    (cond
      ((equal arm :arms)
       (let ((larm-set-torque-limit-p (send self :set-torque-limit-step :larm torque))
             (rarm-set-torque-limit-p (send self :set-torque-limit-step :rarm torque)))
         (and larm-set-torque-limit-p rarm-set-torque-limit-p)))
      (t (send self :set-torque-limit-step arm torque))))
  (:set-torque-limit-step (arm torque)
    (cond
      ((or (equal (send self :get-gripper-type arm) :softhand)
           (equal (send self :get-gripper-type arm) :softhand-v2))
       (send (send self :get-gripper-interface arm) :set-torque-limit-all torque))
      (t
       (ros::ros-warn ":set-torque-limit is invalid for this gripper type")
       nil))))


(defun baxter-init
  (&key (safe t) (type :default-controller) (moveit t)
        (lgripper :softhand) (rgripper :softhand))
  (let (mvit-env mvit-rb)
    (if moveit
      (progn
        (setq mvit-env (instance baxter-moveit-environment))
        (setq mvit-rb (instance baxter-robot :init))))
    (if (not (boundp '*ri*))
      (setq *ri* (instance baxter-softhand-interface :init :type type
                           :moveit-environment mvit-env
                           :moveit-robot mvit-rb
                           :controller-timeout 10
                           :lgripper lgripper
                           :rgripper rgripper)))
    (if (not (boundp '*baxter*))
      (progn
        (if safe
          (setq *baxter* (instance baxter-robot-safe :init))
          (setq *baxter* (instance baxter-robot :init)))
        (send *baxter* :angle-vector (send *ri* :state :potentio-vector))))
    (if (equal lgripper :parallel) (send *ri* :calib-grasp :larm))
    (if (equal rgripper :parallel) (send *ri* :calib-grasp :rarm))))


(provide :baxter-softhand-interface)
