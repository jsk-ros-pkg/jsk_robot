(ros::roseus "spot_kinova")

(load "package://spoteus/spot-interface.l")
(load "package://kinovaeus/kinova-interface.l")
(load "package://spotkinovaeus/spotkinova-util.l")

;; defclass (To see raw definition, replace eval with macroexpand)
(eval `(defclass spot-kinova-interface :super robot-interface
         :slots ((spot-inst :forward ,(mapcar #'car (send spot-interface :methods)))
                 (kinova-inst :forward ,(mapcar #'car (send kinova-interface :methods))))
         :documentation "This is a robot interface class for using spot + kinova.
This class is designed to be able to call both the methods defined in spot-interface and kinova-interface.

The following is priority when calling methods of the same name defined in different classes.
1. spot-kinova-interface 2. robot-interface 3. spot-interface 4. kinova-interface
Please see https://github.com/euslisp/EusLisp/issues/454#issuecomment-863136824
"
         ))

(defmethod spot-kinova-interface
  (:init
   (&key type &rest args)
   (setq spot-inst (instance spot-interface :init))
   (setq kinova-inst (instance kinova-interface :init :type type :prefix-arg "kinova_"))
   (let ()
     (send-super* :init
                  :robot
                  ;; Create robot instance different from *kinova*
                  (case type
                        (:gen3_robotiq_2f_85
                         (instance spot_gen3_robotiq_2f_85-robot :init))
                        (:gen3_robotiq_2f_140
                         (instance spot_gen3_robotiq_2f_140-robot :init))
                        (:gen3_lite_gen3_lite_2f
                         (instance spot_gen3_lite_gen3_lite_2f-robot :init))
                        (t
                         (warning-message 1 "unknown kinova robot type ~A~%" type)))
                  :joint-states-topic "/spot_kinova/joint_states"
                  :groupname "spot_kinova_interface" args))
   (send self :add-controller :head-controller)
   self)
  (:default-controller ()
    (send self :head-controller))
  (:head-controller ()
    (list
     (append
      (cond ((or (derivedp robot spot_gen3_robotiq_2f_140-robot) (derivedp robot spot_gen3_robotiq_2f_85-robot))
             (list
              (cons :controller-action
                    "arm_gen3/kinova_gen3_joint_trajectory_controller/follow_joint_trajectory") ;; @TODO: This is dummy topoic name.
              (cons :controller-state "arm_gen3/kinova_gen3_joint_trajectory_controller/state"))) ;; @TODO: This is dummy topoic name.
            ((derivedp robot spot_gen3_lite_gen3_lite_2f-robot)
             (list
              (cons :controller-action
                    "arm_gen3/kinova_gen3_lite_joint_trajectory_controller/follow_joint_trajectory") ;; @TODO: This is dummy topoic name.
              (cons :controller-state "arm_gen3/kinova_gen3_lite_joint_trajectory_controller/state")))) ;; @TODO: This is dummy topoic name.
      (list (cons :action-type control_msgs::FollowJointTrajectoryAction)
            (cons :joint-names (send-all (send robot :joint-list) :name))))))
  (:angle-vector (av &optional (tm 3000) &rest args)
    (when (send self :simulation-modep)
      (return-from :angle-vector (send-super :angle-vector-simulation av tm :head-controller)))
    (let ((av-orig (send robot :angle-vector)) spot-av kinova-av)
      (send robot :angle-vector av)
      (setq spot-av (send robot :spot-angle-vector))
      (setq kinova-av (send robot :kinova-angle-vector))
      (send robot :angle-vector av-orig)
      (send* kinova-inst :angle-vector kinova-av args)))
  (:angle-vector-raw (av &optional (tm 3000) &rest args)
    (when (send self :simulation-modep)
      (return-from :angle-vector-raw (send-super :angle-vector-simulation av tm :head-controller)))
    (let ((av-orig (send robot :angle-vector)) spot-av kinova-av)
      (send robot :angle-vector av)
      (setq spot-av (send robot :spot-angle-vector))
      (setq kinova-av (send robot :kinova-angle-vector))
      (send robot :angle-vector av-orig)
      (send* kinova-inst :angle-vector-raw kinova-av args)))
  (:angle-vector-sequence (avs &optional (tms (list 3000)) &rest args)
    (when (send self :simulation-modep)
      (return-from :angle-vector-sequence (send-super :angle-vector-sequence avs tms :head-controller)))
    (let ((av-orig (send robot :angle-vector)) spot-avs kinova-avs)
      (setq spot-avs
            (mapcar #'(lambda (av)
                        (send robot :angle-vector av)
                        (send robot :spot-angle-vector))
                    avs))
      (setq kinova-avs
            (mapcar #'(lambda (av)
                        (send robot :angle-vector av)
                        (send robot :kinova-angle-vector))
                    avs))
      (setq robot av-orig)
      (send* kinova-inst :angle-vector-sequence kinova-avs args)))
  (:angle-vector-sequence-raw (avs &optional (tms (list 3000)) &rest args)
    (when (send self :simulation-modep)
      (return-from :angle-vector-sequence-raw (send-super :angle-vector-sequence avs tms :head-controller)))
    (let ((av-orig (send robot :angle-vector)) spot-avs kinova-avs)
      (setq spot-avs
            (mapcar #'(lambda (av)
                        (send robot :angle-vector av)
                        (send robot :spot-angle-vector))
                    avs))
      (setq kinova-avs
            (mapcar #'(lambda (av)
                        (send robot :angle-vector av)
                        (send robot :kinova-angle-vector))
                    avs))
      (setq robot av-orig)
      (send* kinova-inst :angle-vector-sequence-raw kinova-avs args)))
  (:go-pos (&rest args)
    (send* spot-inst :go-pos args))
  (:go-pos-no-wait (&rest args)
    (send* spot-inst :go-pos-no-wait args))
  (:go-wait (&rest args)
    (send* spot-inst :go-wait args))
  (:go-velocity (&rest args)
    (send* spot-inst :go-velocity args))
  (:go-stop (&rest args)
    (send* spot-inst :go-stop args))
  (:gripper (&rest args)
    (send* kinova-inst :gripper args))
  )

(defun spot-kinova-init (&key (type :gen3_robotiq_2f_140) &rest args &allow-other-keys)
  (unless (boundp '*spot-kinova*)
    (setq *spot-kinova* (spot-kinova :type type)))
  (unless (boundp '*ri*)
    (setq *ri* (instance spot-kinova-interface :init :type type))))
