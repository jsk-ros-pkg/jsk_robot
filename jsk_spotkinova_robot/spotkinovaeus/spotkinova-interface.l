(ros::roseus "spot_kinova_interface")
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
   (&key (type :gen3_lite_gen3_lite_2f) &rest args)
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
                    "dummy/follow_joint_trajectory") ;; @TODO: This is dummy topoic name.
              (cons :controller-state "arm_gen3/kinova_gen3_joint_trajectory_controller/state"))) ;; @TODO: This is dummy topoic name.
            ((derivedp robot spot_gen3_lite_gen3_lite_2f-robot)
             (list
              (cons :controller-action
                    "dummy/follow_joint_trajectory") ;; @TODO: This is dummy topoic name.
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
      (send robot :angle-vector av-orig)
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
      (send robot :angle-vector av-orig)
      (send* kinova-inst :angle-vector-sequence-raw kinova-avs args)))
  (:wait-interpolation (&rest args)
    (send* kinova-inst :wait-interpolation args))
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
  (:add-pc-case-collision-object ()
    (let ((cube (make-cube 460 330 200)))
      (send cube :translate #f(-300 0 100))
      (send *co* :add-attached-object cube
            "kinova_base_link"
            :frame-id "kinova_base_link"
            :object-id "pc_case")))
  (:add-insta360-collision-object ()
    (let ((cube (make-cube 50 50 400)))
      (send cube :translate #f(-150 0 300))
      (send *co* :add-attached-object cube
            "kinova_base_link"
            :frame-id "kinova_base_link"
            :object-id "insta360")))
  (:state (&rest args)
    (cond
     ;; For several methods, call spot-interface's :state method.
     ((member (car args)
              (list :metrics :leases :feet :estop :wifi :power-state :battery-states :behvior-faults :system-fault :feedback :power-state-motor-power-state))
      (send* spot-inst :state args))
     (t
      (send-super* :state args))))
  (:spin-once (&rest args)
    (send-super* :spin-once args)
    (send* spot-inst :spin-once args)
    (send* kinova-inst :spin-once args))
  )

(defun spot-kinova-init (&key (type :gen3_lite_gen3_lite_2f) &rest args &allow-other-keys)
  (unless (boundp '*spot-kinova*)
    (setq *spot-kinova* (spot-kinova :type type)))
  (unless (ros::ok)
    (ros::roseus "spot_kinova_interface"))
  (unless (boundp '*ri*)
    (setq *ri* (instance spot-kinova-interface :init :type type)))

  ;; Copied from spot-interface (spot-init)
  ;; This should be done in (spot-interface :init) ?
  (ros::spin-once)
  (send *ri* :spin-once)
  (send *ri* :claim)
  (while (member (send *ri* :state :power-state-motor-power-state) (list 'off nil))
    (unix::sleep 1)
    (ros::ros-info "powering on...")
    (send *ri* :power-on))
  (unless (every #'(lambda (x) (eq x 'made)) (mapcar #'(lambda (x) (cdr (assoc :contact x))) (send *ri* :state :feet)))
      (ros::ros-info "run (send *ri* :stand) to stand the robot"))

  ;; Copied from kinova-interface (kinova-init)
  ;; This should be done in (kinova-interface :init) ?
  (unless (boundp '*co*)
    (ros::ros-warn ";; collision-object-publisher wait for \"arm_gen3/apply_planning_scene\" service for ~A sec~%" 5)
    (if (ros::wait-for-service "arm_gen3/apply_planning_scene" 5)
      (setq *co* (instance collision-object-publisher
                           :init
                           :service-name "arm_gen3/apply_planning_scene"
                           :scene-service "arm_gen3/get_planning_scene"))
      (ros::ros-warn ";; could not find \"arm_gen3/apply_planning_scene\" skip creating *co*~%")))
  (unix:usleep (* 100 1000))
  (send *co* :wipe-all)
  (send *ri* :add-pc-case-collision-object)
  (send *ri* :add-insta360-collision-object)

  *ri*
  )
