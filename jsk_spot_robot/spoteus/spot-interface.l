(ros::roseus "spot")

(require "package://spoteus/spot-utils.l")
(require "package://pr2eus/robot-interface.l")

(ros::roseus-add-srvs "std_srvs")

(defun call-trigger-service (srvname &key (wait nil))
  "Call std_srv/Trigger service"
  (let (r)
    (if wait (ros::wait-for-service srvname))
    (setq r (ros::service-call srvname (instance std_srvs::TriggerRequest :init)))
    (ros::ros-info "Call \"~A\" returns \"~A\"" srvname (send r :message))
    (send r :success)))

(defclass spot-interface
  :super robot-move-base-interface
  :slots ()
  )

(defmethod spot-interface
  (:init
   (&rest args)
   (send-super* :init :robot spot-robot :base-frame-id "base_link" :odom-topic "/odom_combined" :base-controller-action-name nil args)
   )
  (:default-controller () ) ;; spot does not provide any JTA controllers
  (:state
   (&rest args)
   (prog1
       (send-message self robot-interface :state args)
     (case (car args)
       (:angle-vector
	(return-from :state (send robot :angle-vector)))
       (:worldcoords
	(unless joint-action-enable
	  (return-from :state (send self :worldcoords)))
	(return-from :state (send *tfl* :lookup-transform "vision" base-frame-id (ros::time 0)))))))
  ;;
  (:estop-gentle () (call-trigger-service "/spot/estop/gentle"))
  (:estop-hard () (call-trigger-service "/spot/estop/hard"))
  (:claim () "Claim the robot control" (call-trigger-service "/spot/claim"))
  (:release () "Relase the robot control" (call-trigger-service "/spot/release"))
  (:power-on () "Power on the robot" (call-trigger-service "/spot/power_on"))
  (:power-off () "Power off the robot" (call-trigger-service "/spot/power_off"))
  (:self-right () (call-trigger-service "/spot/self_right"))
  (:stand () "Stand the robot up" (call-trigger-service "/spot/stand"))
  (:sit () "Sit the robot down" (call-trigger-service "/spot/sit"))
  (:stop () "Stop the robot in place with minimal motion" (call-trigger-service "/spot/stop"))
  ;;
  (:send-cmd-vel-raw
   (x y d &key (topic-name "/spot/cmd_vel"))
   (when (send self :simulation-modep)
     (return-from :send-cmd-vel-raw t))
    (unless (ros::get-topic-publisher topic-name)
      (ros::advertise topic-name geometry_msgs::Twist 1)
      (unix:sleep 1))
    (let ((msg (instance geometry_msgs::Twist :init)))
      (send msg :linear :x x)
      (send msg :linear :y y)
      (send msg :angular :z d)
      (ros::publish topic-name msg)))
  (:go-velocity
    (x y d ;; [m/sec] [m/sec] [rad/sec]
       &optional (msec 1000) ;; msec is total animation time [msec]
       &key (stop t) (wait t))
    "contorl the robot velocity x([m/sec]) y([m/sec]) d([rad/sec]) msec([msec]). msec is the time to travel."
    (unless wait
      (ros::ros-error ":go-velocity without wait is unsupported")
      (return-from :go-velocity nil))
    (ros::rate 100)
    (let ((start-time (ros::time-now)))
      (while (and (ros::ok)
                  (< (* 1000.0 (send (ros::time- (ros::time-now) start-time) :to-sec)) msec))
        (send self :spin-once)
        (send self :send-cmd-vel-raw x y d)
        (ros::sleep)))
    (when stop
      (send self :send-cmd-vel-raw 0 0 0))
    (ros::rate 10)
    t)

  )

(defun spot-init (&optional (create-viewer))
  (unless (boundp '*spot*) (spot) (send *spot* :reset-pose))
  (unless (ros::ok) (ros::roseus "spot_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)
  (send *ri* :claim)

  (when create-viewer (objects (list *spot*)))
  )
