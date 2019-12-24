;; go-dock.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "fetch_auto_dock_msgs")
(ros::load-ros-manifest "power_msgs")

(defparameter *dock-action* nil)
(defparameter *undock-action* nil)
(defparameter *spots* nil)
(defparameter *dock-spot* "/eng2/7f/room73B2-fetch-dock-front")
(defparameter *is-charging* nil)
(defparameter *tfl* (instance ros::transform-listener :init))


(defun get-spot-coords (name)
  (unless *spots*
    (setq *spots* (one-shot-subscribe "/spots_marker_array" visualization_msgs::MarkerArray)))
  (let ((spot-coords nil) (frame-id nil))
    (dolist (x (send *spots* :markers))
      (if (equal (send x :text) name)
        (progn
          (setq spot-coords (send x :pose))
          (setq frame-id (send (send x :header) :frame_id)))))
    (send (send spot-coords :position) :z 0)
    (setq spot-coords (ros::tf-pose->coords spot-coords))
    (cons spot-coords frame-id)))

(defun simple-dock ()
  (unless *dock-action*
    (setq *dock-action*
          (instance ros::simple-action-client :init
                    "/dock" fetch_auto_dock_msgs::DockAction)))
  (unless (send *dock-action* :wait-for-server 5)
    (ros::ros-error "/dock action server is not started")
    (return-from dock nil))
  (let* ((timestamp (ros::time-now))
         (cret (get-spot-coords *dock-spot*))
         (frame-to-dock (car cret))
         (frame-id (cdr cret))
         (lret (send *tfl* :wait-for-transform "base_link" frame-id timestamp 5))
         (base-to-frame (send *tfl* :lookup-transform "base_link" frame-id timestamp))
         (goal-pose (ros::coords->tf-pose (send frame-to-dock :transform base-to-frame :world)))
         (pose-msg (instance geometry_msgs::PoseStamped :init))
         (dock-action-goal (instance fetch_auto_dock_msgs::DockActionGoal :init)))
    (send pose-msg :header :stamp timestamp)
    (send pose-msg :header :frame_id "base_link")
    (send pose-msg :pose goal-pose)
    (send dock-action-goal :goal :dock_pose pose-msg)
    (send *dock-action* :send-goal dock-action-goal)
    (unless (send *dock-action* :wait-for-result :timeout 60)
      (send *dock-action* :cancel-all-goals)
      (ros::ros-error "No result returned from /dock action server")
      (return-from simple-dock nil))
    (send (send *dock-action* :get-result) :docked)))

(defun dock ()
  ;; look down
  (unless (boundp '*ri*)
    (require :fetch-interface "package://fetcheus/fetch-interface.l")
    (fetch-init))
  (send *fetch* :angle-vector (send *ri* :state :potentio-vector))
  (send *fetch* :head :look-at (float-vector 800 0 500))
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
  (let* ((ret (get-spot-coords name))
         (goal-pose (car ret))
         (frame-id (cdr ret)))
    (when relative-coords
      (setq goal-pose (send goal-pose :transform relative-coords :world)))
    (send *ri* :clear-costmap)
    (send *ri* :move-to goal-pose :frame-id frame-id)))

(defun auto-dock (&key (n-trial 1))
  (let ((success nil))
    (dotimes (i n-trial)
      (go-to-spot *dock-spot* (make-coords :pos #f(0 -800 0)))
      (setq success (dock))
      (when success (return-from auto-dock success)))
    success))
