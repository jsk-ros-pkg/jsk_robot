(require "package://kinovaeus/gen3-lite-utils.l")
(require "package://pr2eus/robot-interface.l")

;; TODO
;; Enable to use moveit

(defclass gen3-lite-interface
  :super robot-interface
  :slots (gripper-action)
  )

(defmethod gen3-lite-interface
  (:init
   (&rest args)
   (send-super* :init :robot gen3_lite_gen3_lite_2f-robot :joint-states-topic "/my_gen3_lite/joint_states" :groupname "gen3_lite_interface" args)
   (send self :add-controller :rarm-controller)
   (setq gripper-action
         (instance ros::simple-action-client :init
                   "/my_gen3_lite/gen3_lite_2f_gripper_controller/gripper_cmd"
                   control_msgs::GripperCommandAction
                   :groupname groupname))
   )
  (:default-controller () (send self :rarm-controller))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "my_gen3_lite/gen3_lite_joint_trajectory_controller/follow_joint_trajectory")
     (cons :controller-state "my_gen3_lite/gen3_lite_joint_trajectory_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "joint_1" "joint_2" "joint_3" "joint_4" "joint_5" "joint_6")))))
  (:go-grasp (&key (pos 0.0) (wait t))
    (when (send self :simulation-modep)
      (send robot :right_finger_bottom_joint :joint-angle
            (rad2deg pos))
      (send robot :right_finger_tip_joint :joint-angle
            (rad2deg (+ (* -0.676 pos) 0.149)))
      (send robot :left_finger_bottom_joint :joint-angle
            (rad2deg (* -1 pos)))
      (send robot :left_finger_tip_joint :joint-angle
            (rad2deg (+ (* -0.676 pos) 0.149)))
      (return-from :go-grasp t))
    (let ((pos-max 0.95) (pos-min -0.05))
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
    (send* self :go-grasp :pos -0.05 args))
  (:stop-grasp
    (&rest args &key &allow-other-keys)
    (send* self :go-grasp :pos 0.95 args))
  )

(defun gen3-lite-init (&optional (create-viewer))
  (unless (boundp '*gen3-lite*) (gen3-lite) (send *gen3-lite* :reset-pose))
  (unless (ros::ok) (ros::roseus "gen3_lite_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance gen3-lite-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (when create-viewer (objects (list *gen3-lite*)))
  )
