;; go-dock.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "fetch_auto_dock_msgs")
(ros::load-ros-manifest "power_msgs")

(defparameter *dock-action* nil)
(defparameter *undock-action* nil)
(defparameter *spots* nil)
(defparameter *dock-spot* "/eng2/7f/room73B2-fetch-dock-front")
(defparameter *is-charging* nil)

(defun simple-dock ()
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

(defun dock ()
  ;; look down
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :head :look-at (float-vector 1500 0 500))
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (simple-dock))

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

(defun go-to-spot (name &optional (relative-coords (make-coords)))
  ;; undock if fetch is docking
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (dotimes (i 3)
    (let ((msg (one-shot-subscribe "/battery_state" power_msgs::batterystate :timeout 1500)))
      ;; You may fail to subscribe /battery_state
      ;; because of message md5 difference between melodic and indigo.
      (unless msg
        (ros::ros-warn "Failed to subscribe /battery_state")
        (ros::ros-warn "Skip undocking, so please make sure that Fetch is already undocked.")
        (return))
      (setq *is-charging* (send msg :is_charging))
      (if *is-charging*
        (progn (undock)))
      (unless *is-charging* (return))
      (if (eq i 2) (progn (send *ri* :speak "Fail to undock") (ros::ros-error "Fail to undock")))))
  ;; go to spot
  (unless *spots*
    (setq *spots* (one-shot-subscribe "/spots_marker_array" visualization_msgs::MarkerArray)))
  (let ((goal-pose nil) (frame-id nil))
    (dolist (x (send *spots* :markers))
      (if (equal (send x :text) name)
        (progn
          (setq goal-pose (send x :pose))
          (setq frame-id (send (send x :header) :frame_id)))))
    (send (send goal-pose :position) :z 0)
    (setq goal-pose (send (ros::tf-pose->coords goal-pose) :transform relative-coords :world))
    (send *ri* :move-to goal-pose :frame-id frame-id)))

(defun auto-dock ()
  (go-to-spot "/eng2/7f/room73B2-fetch-dock-front" (make-coords :pos #f(0 -1500 0)))
  (dock))
