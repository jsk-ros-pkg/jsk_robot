(require "package://cobottaeus/cobotta-utils.l")
(require "package://pr2eus/robot-interface.l")

(defclass cobotta-oss-interface
  :super robot-interface
  :slots ())

(defmethod cobotta-oss-interface
  (:init
    (&rest args)
    (prog1
      (send-super* :init :robot cobotta-robot :joint-states-topic "/cobotta/joint_states"
                   :groupname "cobotta_interface" args)
      (send self :add-controller :rarm-controller)))
  (:default-controller () (send self :rarm-controller))
  (:rarm-controller ()
    (list
      (list
        (cons :controller-action "cobotta/arm_controller/follow_joint_trajectory")
        (cons :controller-state "cobotta/arm_controller/state")
        (cons :action-type control_msgs::FollowJointTrajectoryAction)
        (cons :joint-names (list "joint_1" "joint_2" "joint_3" "joint_4" "joint_5" "joint_6"))))))

(defun cobotta-oss-init (&optional (create-viewer))
  (unless (boundp '*cobotta*) (cobotta) (send *cobotta* :reset-pose))
  (unless (ros::ok) (ros::roseus "cobotta_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance cobotta-oss-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (when create-viewer (objects (list *cobotta*))))
