(ros::roseus "go1")

(require "package://unitreeeus/go1-utils.l")
(require "package://pr2eus/robot-interface.l")

(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "unitree_legged_msgs")

;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/705
(defun call-trigger-service (srvname &key (wait nil) (timeout -1) (persistent t))
  "Call std_srv/Trigger service"
  (let (r)
    (when (ros::wait-for-service srvname (if wait timeout 0))
      (setq r (ros::service-call srvname (instance std_srvs::TriggerRequest :init) persistent))
      (ros::ros-debug "Call \"~A\" returns \"~A\"" srvname (send r :message))
      (send r :success))))

;; waiting for 1.7.5
;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/662
(defun ros::create-quaternion-from-rpy (roll pitch yaw)
  "return #F(w x y z)"
  (let ((sin-roll  (sin (* roll  0.5))) (cos-roll  (cos (* roll  0.5)))
        (sin-pitch (sin (* pitch 0.5))) (cos-pitch (cos (* pitch 0.5)))
        (sin-yaw   (sin (* yaw   0.5))) (cos-yaw   (cos (* yaw   0.5))))
    (float-vector (+ (* cos-roll cos-pitch cos-yaw) (* sin-roll sin-pitch sin-yaw))
                  (- (* sin-roll cos-pitch cos-yaw) (* cos-roll sin-pitch sin-yaw))
                  (+ (* cos-roll sin-pitch cos-yaw) (* sin-roll cos-pitch sin-yaw))
                  (- (* cos-roll cos-pitch sin-yaw) (* sin-roll sin-pitch cos-yaw)))))

(defclass go1-interface
  :super robot-move-base-interface
  :slots (body-pose-topic high-cmd-topic)
  )

(defmethod go1-interface
  (:init
   (&rest args)
   (prog1
       (send-super* :init :robot go1_description-robot :base-frame-id "base_lk" :odom-topic "/odom_combined" :base-controller-action-name nil args)
     (setq body-pose-topic "/go1/body_pose")
     (ros::advertise body-pose-topic geometry_msgs::Pose 1)
     (setq cmd-vel-topic "/go1/cmd_vel")
     (ros::advertise cmd-vel-topic geometry_msgs::Twist 1)
     ;; for debugging
     (setq high-cmd-topic "/high_cmd")
     (ros::advertise high-cmd-topic unitree_legged_msgs::HighCmd 1)
     ))
  (:default-controller () ) ;; go1 does not provide any JTA controllers
  (:stand () "Stand the robot up" (call-trigger-service "/go1/stand"))
  (:sit () "Sit the robot down" (call-trigger-service "/go1/sit"))
  (:send-cmd-vel-raw
   (x y d)
   (when (send self :simulation-modep)
     (return-from :send-cmd-vel-raw t))
    (let ((msg (instance geometry_msgs::Twist :init)))
      (send msg :linear :x x)
      (send msg :linear :y y)
      (send msg :angular :z d)
      (ros::publish cmd-vel-topic msg)))
  (:go-velocity
    (x y d ;; [m/sec] [m/sec] [rad/sec]
       &optional (msec 1000) ;; msec is total animation time [msec]
       &key (stop t) (wait t))
    "control the robot velocity x([m/sec]) y([m/sec]) d([rad/sec]) msec([msec]). msec is the time to travel."
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
  (:body-pose
   ;; pose expected to get 3 elements float-vector (r p y), or eus coords.
    (pose)
    (when (send self :simulation-modep)
      (return-from :body-pose t))
    (if (or (vectorp pose) (listp pose))
        (progn
          (ros::ros-debug "Got r p y float-vector or list as args.")
          (let ((pose-msg (instance geometry_msgs::Pose :init))
                (quaternion-msg (ros::create-quaternion-msg-from-rpy (elt pose 0) (elt pose 1) (elt pose 2))))
            (send pose-msg :orientation quaternion-msg)
            (ros::publish body-pose-topic pose-msg)))
        (progn
          (ros::ros-debug "Got coords variable as args.")
          (let ((pose-msg (ros::coords->tf-pose pose)))
            (ros::publish body-pose-topic pose-msg))))
    )
  ;;
  (:led (id rgb) ;; not working?
   (let ((msg (instance unitree_legged_msgs::HighCmd :init))
         (led (instance unitree_legged_msgs::Led :init :r (elt rgb 0) :g (elt rgb 1) :b (elt rgb 2))))
     (setf (elt (send msg :led) id) led)
     (ros::publish high-cmd-topic msg)
     ))
  (:recovery-stand ()
   (let ((msg (instance unitree_legged_msgs::HighCmd :init)))
     (send msg :mode 8)
     (ros::publish high-cmd-topic msg)))
  (:backflip ()
   (let ((msg (instance unitree_legged_msgs::HighCmd :init)))
     (send msg :mode 9)
     (ros::publish high-cmd-topic msg)))
  (:jump-yaw ()
   (let ((msg (instance unitree_legged_msgs::HighCmd :init)))
     (send msg :mode 10)
     (ros::publish high-cmd-topic msg)))
  (:straight-hand ()
   (let ((msg (instance unitree_legged_msgs::HighCmd :init)))
     (send msg :mode 11)
     (ros::publish high-cmd-topic msg)
     ))
  (:dance1 ()
   (let ((msg (instance unitree_legged_msgs::HighCmd :init)))
     (send msg :mode 12)
     (ros::publish high-cmd-topic msg)
     ))
  (:dance2 ()
   (let ((msg (instance unitree_legged_msgs::HighCmd :init)))
     (send msg :mode 13)
     (ros::publish high-cmd-topic msg)
     ))
  (:two-leg-stand ()
   (let ((msg (instance unitree_legged_msgs::HighCmd :init)))
     (send msg :mode 14)
     (ros::publish high-cmd-topic msg)
     ))
  )

(defun go1-init ()
  (unless (boundp '*go1*) (go1))
  (unless (ros::ok) (ros::roseus "go1_eus_interface"))
  (unless (boundp '*ri*) (setq *ri* (instance go1-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)
  )
