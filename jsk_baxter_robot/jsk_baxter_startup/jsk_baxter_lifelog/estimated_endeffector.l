(load "package://baxtereus/baxter-interface.l")
(ros::roseus "baxter_endeffector_wrench")
(ros::advertise "/robot/end_effector/left/wrench" geometry_msgs::WrenchStamped 1)
(ros::advertise "/robot/end_effector/right/wrench" geometry_msgs::WrenchStamped 1)

(if (not (boundp '*baxter*))
    (setq *baxter* (instance baxter-robot :init)))

(ros::rate 20)

(defun right-state-cb(msg)
  (state-cb "/robot/end_effector/right/wrench" :rarm (send msg :wrench))
  )

(defun left-state-cb(msg)
  (state-cb "/robot/end_effector/left/wrench" :larm (send msg :wrench))
  )

(defun state-cb(advertise-topic arm wrench-msg)
  (let ((msg (instance geometry_msgs::WrenchStamped :init)))
    (send msg :wrench wrench-msg)
    (send msg :header :stamp (ros::time-now))
    (send msg :header :frame_id (send (send (send *baxter* arm :end-coords) :parent) :name))
    (ros::publish advertise-topic msg))
  )


(ros::subscribe "/robot/limb/left/endpoint_state" baxter_core_msgs::EndpointState #'left-state-cb)
(ros::subscribe "/robot/limb/right/endpoint_state" baxter_core_msgs::EndpointState #'right-state-cb)

(ros::spin)