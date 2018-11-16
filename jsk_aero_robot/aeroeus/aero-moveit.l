(ros::load-ros-manifest "pr2eus_moveit")

(require :robot-moveit "package://pr2eus_moveit/euslisp/robot-moveit.l")
(require :aero-interface "package://aeroeus/aero-interface.l")

(defclass aero-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod aero-moveit-environment
  (:init
   (&key args)
   (send-super* :init :robot (instance aero-lifter-ik :init)
                :frame-id "base_link" args))
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "rarm")
               (cons :target-link
                     (send self :search-link-from-name "r_hand_link"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "larm")
               (cons :target-link
                     (send self :search-link-from-name "l_hand_link"))
               (cons :joint-list (send robot :larm :joint-list))
               )
         (list :rarm-with-waist
               (cons :group-name "rarm_with_waist")
               (cons :target-link
                     (send self :search-link-from-name "r_hand_link"))
               (cons :joint-list (append (subseq (send robot :torso :joint-list) 0 3)
                                         (send robot :rarm :joint-list)))
               )
         (list :larm-with-waist
               (cons :group-name "larm_with_waist")
               (cons :target-link
                     (send self :search-link-from-name "l_hand_link"))
               (cons :joint-list (append (subseq (send robot :torso :joint-list) 0 3)
                                         (send robot :larm :joint-list)))
               )
         (list :rarm-with-torso
               (cons :group-name "rarm_with_torso")
               (cons :target-link
                     (send self :search-link-from-name "r_hand_link"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :rarm :joint-list)))
               )
         (list :larm-with-torso
               (cons :group-name "larm_with_torso")
               (cons :target-link
                     (send self :search-link-from-name "l_hand_link"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :larm :joint-list)))
               )
         (list :both-arms
               (cons :group-name "both_arms")
               (cons :target-link
                     (list (send self :search-link-from-name "l_hand_link")
                           (send self :search-link-from-name "r_hand_link")))
               (cons :joint-list (append (send robot :larm :joint-list)
                                         (send robot :rarm :joint-list)))
               )
         (list :both-arms-with-waist
               (cons :group-name "both_arms_with_waist")
               (cons :target-link
                     (list (send self :search-link-from-name "l_hand_link")
                           (send self :search-link-from-name "r_hand_link")))
               (cons :joint-list (append (subseq (send robot :torso :joint-list) 0 3)
                                         (send robot :larm :joint-list)
                                         (send robot :rarm :joint-list)))
               )
#| ;; sample of aero_moveit_config/config/AeroUpperRobot.srdf
    <group name="rarm">
        <chain base_link="body_link" tip_link="r_hand_link" />
    </group>
    <group name="larm">
        <chain base_link="body_link" tip_link="l_hand_link" />
    </group>
    <group name="both_arms">
        <group name="larm" />
        <group name="rarm" />
    </group>
    <group name="waist">
        <chain base_link="waist_link" tip_link="body_link" />
    </group>
    <group name="lifter">
        <chain base_link="lifter_base_link" tip_link="lifter_top_link" />
    </group>
    <group name="rarm_with_waist">
        <group name="rarm" />
        <group name="waist" />
    </group>
    <group name="larm_with_waist">
        <group name="larm" />
        <group name="waist" />
    </group>
    <group name="both_arms_with_waist">
        <group name="larm" />
        <group name="rarm" />
        <group name="waist" />
    </group>
    <group name="rarm_with_torso">
        <group name="rarm" />
        <group name="waist" />
        <group name="lifter" />
    </group>
    <group name="larm_with_torso">
        <group name="larm" />
        <group name="waist" />
        <group name="lifter" />
    </group>
    <group name="rarm_with_lifter">
        <group name="rarm" />
        <group name="lifter" />
    </group>
    <group name="larm_with_lifter">
        <group name="larm" />
        <group name="lifter" />
    </group>
    <group name="head">
        <chain base_link="body_link" tip_link="head_link" />
    </group>
|#
         ))

(provide :aero-moveit "aero-moveit.l")

#|
(let* ((tm (ros::time-now))
       (rpose (ros::coords->tf-pose-stamped
               (send *aero* :r_hand_link_lk)
               "base_link"))
       (lpose (ros::coords->tf-pose-stamped
               (send *aero* :l_hand_link_lk)
               "base_link"))
       (msg
        (instance moveit_msgs::GetPositionIKRequest :init
                  :ik_request
                  (instance moveit_msgs::PositionIKRequest :init
                            :group_name "both_arms_with_waist"
                            :avoid_collisions t
                            :robot_state (instance moveit_msgs::RobotState :init)
                            :attempts 0
                            :timeout (ros::time 0.05)
                            ;; :pose_stamped msg
                            ;; :ik_link_name
                            :ik_link_names (list "r_hand_link" "l_hand_link")
                            :pose_stamped_vector (list rpose lpose)
                            )
                  )
        ))
  (ros::service-call "/compute_ik" msg)
  )
#|
