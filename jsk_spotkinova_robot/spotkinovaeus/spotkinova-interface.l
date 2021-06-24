(ros::roseus "spot_kinova")

(load "package://spoteus/spot-interface.l")
(load "package://kinovaeus/kinova-interface.l")
(load "package://spotkinovaeus/spotkinova-util.l")

;; Do not forward methods defined in spot-kinova-interface class
(defvar *non-forward-methods*
  (list :init :default-controller :head-controller :angle-vector :angle-vector-raw :angle-vector-sequence :angle-vector-sequence-raw))

(eval `(defclass spot-kinova-interface :super robot-interface
         :slots ((spot-inst :forward (remove-if #'(lambda (x) (member x ,*non-forward-methods*)) ,(mapcar #'car (send spot-interface :methods))))
                 (kinova-inst :forward (remove-if #'(lambda (x) (member x ,*non-forward-methods*)) ,(mapcar #'car (send kinova-interface :methods)))))))

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
  (:angle-vector (av &optional tm &rest args)
    (when (send self :simulation-modep)
      (return-from :angle-vector (send-super :angle-vector-simulation av tm :head-controller)))
    (let* ((kinova-av (coerce (nthcdr 12 (coerce av cons)) float-vector))
           (spot-av-list (butlast (coerce av cons) 6)))
      (setq kinova-av (send* kinova-inst :angle-vector kinova-av args))
      (coerce (append spot-av-list (coerce kinova-av cons)) float-vector)))
  (:angle-vector-raw (av &optional tm &rest args)
    (when (send self :simulation-modep)
      (return-from :angle-vector-raw (send-super :angle-vector-simulation av tm :head-controller)))
    (let* ((kinova-av (coerce (nthcdr 12 (coerce av cons)) float-vector))
           (spot-av (coerce (butlast (coerce av cons) 6) float-vector)))
      (setq kinova-av (send* kinova-inst :angle-vector-raw kinova-av args))
      (coerce (append (coerce spot-av cons) (coerce kinova-av cons)) float-vector)))
  (:angle-vector-sequence (avs &optional tms &rest args)
    (when (send self :simulation-modep)
      (return-from :angle-vector-sequence (send-super :angle-vector-sequence avs tms :head-controller)))
    (let* ((kinova-avs
            (mapcar #'(lambda (av) (coerce (nthcdr 12 (coerce av cons)) float-vector)) avs))
           (spot-avs
            (mapcar #'(lambda (av) (coerce (butlast (coerce av cons) 6) float-vector)) avs)))
      (setq kinova-avs (send* kinova-inst :angle-vector-sequence kinova-avs args))
      (mapcar #'(lambda (spot-av kinova-av)
                  (coerce (append (coerce spot-av cons) (coerce kinova-av cons)) float-vector))
              spot-avs kinova-avs)))
  (:angle-vector-sequence-raw (avs &optional tms &rest args)
    (when (send self :simulation-modep)
      (return-from :angle-vector-sequence-raw (send-super :angle-vector-sequence avs tms :head-controller)))
    (let* ((kinova-avs
            (mapcar #'(lambda (av) (coerce (nthcdr 12 (coerce av cons)) float-vector)) avs))
           (spot-avs
            (mapcar #'(lambda (av) (coerce (butlast (coerce av cons) 6) float-vector)) avs)))
      (setq kinova-avs (send* kinova-inst :angle-vector-sequence-raw kinova-avs args))
      (mapcar #'(lambda (spot-av kinova-av)
                  (coerce (append (coerce spot-av cons) (coerce kinova-av cons)) float-vector))
              spot-avs kinova-avs)))
  )

(defun spot-kinova-init (&key (type :gen3_robotiq_2f_140) &rest args &allow-other-keys)
  (unless (boundp '*spot-kinova*)
    (setq *spot-kinova* (spot-kinova :type type)))
  (unless (boundp '*ri*)
    (setq *ri* (instance spot-kinova-interface :init :type type))))
