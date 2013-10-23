(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "actionlib")

(ros::roseus "test_nao_controller")

(defun test-body-stiffness-enable ()
  (ros::wait-for-service "body_stiffness/enable")
  (setq req (instance std_srvs::EmptyRequest :init))
  (setq res (ros::service-call "body_stiffness/enable" req))
)

(defun test-body-stiffness-disable ()
  (ros::wait-for-service "body_stiffness/disable")
  (setq req (instance std_srvs::EmptyRequest :init))
  (setq res (ros::service-call "body_stiffness/disable" req))
)
