(require "package://kinovaeus/gen3-utils.l")
(require "package://pr2eus/robot-interface.l")

;; TODO
;; Enable to use moveit

(defclass gen3-interface
  :super robot-interface
  :slots (gripper-action)
  )

(defmethod gen3-interface
  (:init
   (&rest args)
   (send-super* :init :robot gen3_robotiq_2f_85-robot :joint-states-topic "/my_gen3/joint_states" :groupname "gen3_interface" args)
   (send self :add-controller :rarm-controller)
   (setq gripper-action
         (instance ros::simple-action-client :init
                   "/my_gen3/robotiq_2f_85_gripper_controller/gripper_cmd"
                   control_msgs::GripperCommandAction
                   :groupname groupname))
   )
  (:default-controller () (send self :rarm-controller))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory")
     (cons :controller-state "my_gen3/gen3_joint_trajectory_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "joint_1" "joint_2" "joint_3" "joint_4" "joint_5" "joint_6" "joint_7")))))
  (:go-grasp (&key (pos 0.0) (wait t))
    (when (send self :simulation-modep)
      (let ((pos-deg (rad2deg pos)))
        (send robot :finger_joint :joint-angle pos-deg)
        (send robot :left_inner_finger_joint :joint-angle (* -1 pos-deg))
        (send robot :left_inner_knuckle_joint :joint-angle pos-deg)
        (send robot :right_inner_finger_joint :joint-angle (* -1 pos-deg))
        (send robot :right_inner_knuckle_joint :joint-angle pos-deg)
        (send robot :right_outer_knuckle_joint :joint-angle pos-deg))
      (return-from :go-grasp t))
    (let ((pos-max 0.8) (pos-min 0.0))
      (when (or (< pos pos-min) (> pos pos-max))
      (ros::ros-warn (format nil ":pos ~A is out of range." pos))
      (setq pos (max pos-min (min pos pos-max)))))
    (let (goal result)
      (setq goal (instance control_msgs::GripperCommandActionGoal :init))
      (send goal :goal :command :position pos)
      (send gripper-action :send-goal goal)
      (when wait (send gripper-action :wait-for-result))
      (setq result (send gripper-action :get-result))
      result))
  (:start-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0.8 args))
  (:stop-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0.0 args))
  )

(defun gen3-init (&optional (create-viewer))
  (unless (boundp '*gen3*) (gen3) (send *gen3* :reset-pose))
  (unless (ros::ok) (ros::roseus "gen3_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance gen3-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (when create-viewer (objects (list *gen3*)))
  )
