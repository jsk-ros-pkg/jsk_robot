(ros::load-ros-manifest "fetch_auto_dock_msgs")

(ros::roseus "fetch_dock")

(defparameter *dock-action* nil)

(defun dock ()
  (unless *dock-action*
    (setq *dock-action*
          (instance ros::simple-action-client :init
                    "/dock" fetch_auto_dock_msgs::DockAction)))
  (unless (send *dock-action* :wait-for-server 5)
    (ros::ros-error "/dock action server is not started")
    (return-from dock nil))

   (send *dock-action* :send-goal
        (instance fetch_auto_dock_msgs::DockActionGoal :init))
  (unless (send *dock-action* :wait-for-result :timeout 60)
    (ros::ros-error "No result returned from /dock action server")
    (return-from dock nil))
  (send (send *dock-action* :get-result) :docked))

(dock)
(exit)

