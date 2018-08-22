;; go-dock.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "fetch_auto_dock_msgs")
(ros::load-ros-manifest "power_msgs")

(require :fetch-interface "package://fetcheus/fetch-interface.l")

(unless (boundp '*ri*)
  (fetch-init))

(defparameter *dock-action* nil)
(defparameter *undock-action* nil)
(defparameter *spots* (one-shot-subscribe "/spots_marker_array" visualization_msgs::MarkerArray))
(defparameter *dock-spot* "/eng2/7f/room73B2-fetch-dock-front")
(defparameter *is-charging* nil)

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

(defun go-to-spot (name)
  ;; undock if fetch is docking
  (dotimes (i 3)
    (setq *is-charging* (send (one-shot-subscribe "/battery_state" power_msgs::batterystate) :is_charging))
    (if *is-charging*
      (progn (undock)))
    (unless *is-charging*
      (return))
    (if (eq i 2) (progn (send *ri* :speak "Fail to undock") (ros::ros-error "Fail to undock"))))
  ;; go to spot
  (let ((dock-pose nil))
    (dolist (x (send *spots* :markers)) (if (equal (send x :text) name) (setq dock-pose (send x :pose))))
    (send (send dock-pose :position) :y (- (send (send dock-pose :position) :y) 0.5))
    (send *ri* :move-to (ros::tf-pose->coords dock-pose))))

(defun auto-dock ()
  (go-to-spot  "/eng2/7f/room73B2-fetch-dock-front")
  (dock))
