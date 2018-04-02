;; go-dock.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "fetch_go_dock")

(ros::load-ros-manifest "fetch_auto_dock_msgs")

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(defparameter *dock-action* nil)
(defparameter *undock-action* nil)
(defparameter *scene* (make-eng2-scene))

(defparameter *dock-spot* "/eng2/7f/room73B2-fetch-dock-front")

(defun dock ()
  (unless *dock-action*
    (setq *dock-action*
          (instance ros::simple-action-client :init
                    "/dock" fetch_auto_dock_msgs::DockAction)))
  (unless (send *dock-action* :wait-for-server 5)
    (ros::ros-error "/dock action server is not started")
    (return-from dock nil))

  ;; look down
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :head :look-at (float-vector 1500 0 500))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *dock-action* :send-goal
        (instance fetch_auto_dock_msgs::DockActionGoal :init))
  (unless (send *dock-action* :wait-for-result :timeout 60)
    (ros::ros-error "No result returned from /dock action server")
    (return-from dock nil))
  (send (send *dock-action* :get-result) :docked))

(defun undock ()
  (unless *undock-action*
    (setq *undock-action*
          (instance ros::simple-action-client :init
                    "/undock" fetch_auto_dock_msgs::UndockAction)))
  (unless (send *undock-action* :wait-for-server 5)
    (ros::ros-error "/undock action server is not started")
    (return-from undock nil))

  (send *undock-action* :send-goal
        (instance fetch_auto_dock_msgs::UndockActionGoal :init))
  (unless (send *undock-action* :wait-for-result :timeout 60)
    (ros::ros-error "No result returned from /undock action server")
    (return-from undock nil))
  (send (send *undock-action* :get-result) :undocked))

(defun go-to-dock ()
  (send *ri* :move-to (send *scene* :spot *dock-spot*)))

(defun auto-dock ()
  (unless (boundp '*ri*)
    (fetch-init))
  (send *ri* :speak "auto docking")
  (go-to-dock)
  (dock))

(auto-dock)
(exit)
