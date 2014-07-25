(load "package://pr2eus/robot-interface.l")
(require :baxter "package://baxtereus//baxter.l")
(ros::load-ros-manifest "control_msgs")

(defclass baxter-interface
  :super robot-interface
  :slots ())
(defmethod baxter-interface
  (:init (&rest args)
   (prog1 (send-super :init :robot baxter-robot :joint-states-topic "/robot/joint_states")
     (send self :add-controller :larm-controller)
     (send self :add-controller :rarm-controller)
     ))
  (:default-controller ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)))
  (:larm-controller ()
   (list
    (list
     (cons :controller-action "/robot/limb/left/follow_joint_trajectory")
     (cons :controller-state "/robot/limb/left/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "left_s0" "left_s1" "left_e0" "left_e1" "left_w0" "left_w1")))))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "/robot/limb/right/follow_joint_trajectory")
     (cons :controller-state "/robot/limb/right/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "right_s0" "right_s1" "right_e0" "right_e1" "right_w0" "right_w1")))))
  )

(defmethod baxter-interface
  (:send-ros-controller
   (action joint-names starttime trajpoints)
   (when (send self :simulation-modep)
     (return-from :send-ros-controller nil))
   (if (and warningp
            (yes-or-no-p (format nil "~C[3~CmAre you sure to move the real robot? (~A) ~C[0m" #x1b 49 (send action :name) #x1b)))
       (return-from :send-ros-controller nil))
   (let* ((goal (send action :make-goal-instance))
          (goal-points nil)
          (st (ros::time+ (ros::time-now) (ros::time starttime)))
          (joints (mapcar #'(lambda (x)
                              (send robot (intern (string-upcase x) *keyword-package*)))
                          joint-names)))
     (send goal :header :seq 1)
     (send goal :header :stamp st)
     (send goal :goal :trajectory :joint_names joint-names)
     ;; (send goal :goal :trajectory :header :stamp st)
     (dolist (trajpt trajpoints)
       (let* ((all-positions (elt trajpt 0))
              (all-velocities (elt trajpt 1))
              (duration (elt trajpt 2))
              (positions (instantiate float-vector (length joint-names)))
              (velocities (instantiate float-vector (length joint-names))))
         (dotimes (i (length joints))
           (let* ((joint (elt joints i))
                  (id (position joint (send robot :joint-list)))
                  p v)
             (setq p (elt all-positions id)
                   v (elt all-velocities id))
             (cond
              ((derivedp joint rotational-joint)
               (setq p (deg2rad p))
               (setq v (deg2rad v)))
              (t
               (setq p (* 0.001 p))
               (setq v (* 0.001 v))))
             (setf (elt positions i) p)
             (setf (elt velocities i) v)))
         (push (instance trajectory_msgs::JointTrajectoryPoint
                         :init
                         :positions positions
                         :velocities velocities
                         :time_from_start (ros::time duration))
               goal-points)
         ))
     (send self :spin-once)
     (send goal :goal :trajectory :points goal-points)
     (send action :send-goal goal)
     ))
  )

(In-package "ROS")
(defvar *wait-interpolation-flag*)
(defmethod ros::simple-action-client
  (:action-result-cb
   (msg)
   (let (dummy-msg)
     (ros::ros-debug "[~A] reuslt-cb ~A ~A" name-space (send msg :status :goal_id :id) (goal-status-to-string (send msg :status :status)))
     (send comm-state :update-result msg)
     (ros::ros-debug "[~A] reuslt-cb ~A -> state ~A" name-space (send msg :status :goal_id :id) (send comm-state :state))
     ;; transition to goal state
     ;;(setq dummy-msg (instance actionlib_msgs::GoalStatusArray :init))
     ;;(send dummy-msg :status_list (list (send msg :status)))
     ;;(send dummy-msg :header (send msg :header))
     ;; (send self :goal-status-cb dummy-msg)
     (setq *wait-interpolation-flag* t)
     ))
  (:wait-for-result
   (&key (timeout 0) ;; sec
         (return-if-server-down) (maximum-status-interval 5))
   (let ((start (ros::time-now))
         (result-topic (format nil "~A/result" name-space)))
     (ros::ros-debug "[~A] :wait-for-result ~A, timeout=~A" name-space (if goal_id (send goal_id :id)) timeout)
     (ros::rate 10)
     (while (ros::ok)
       (ros::ros-debug "[~A] :wait-for-result ~A ~A" name-space (simple-goal-states-to-string simple-state) (send comm-state :state))
       (if (= ros::simple-state ros::*simple-goal-state-done*)
	   (return-from :wait-for-result nil))
       (if *wait-interpolation-flag*
	   (return-from :wait-for-result (setq *wait-interpolation-flag* nil)))
       (if (> timeout 0)
           (let* ((tm (ros::time- (ros::time-now) start)))
             (if (> (send tm :to-sec) timeout) (return-from :wait-for-result nil))))
       (when return-if-server-down
         (when (= (ros::get-num-publishers result-topic) 0)
           (ros::ros-error "[~A] Unexpected returns from :wait-for-result : no publishers found for ~A" name-space result-topic)
           (return-from :wait-for-result nil))
         (when (and status-stamp maximum-status-interval (> (send (ros::time- (ros::time-now) status-stamp) :to-sec) maximum-status-interval))
           (ros::ros-error "[~A] Unexpected returns from :wait-for-result : status did not received for 5 seconds" name-space)
           (return-from :wait-for-result nil)))
       (send self :spin-once)
       (ros::sleep))
     (ros::ros-debug "[~A] :wait-for-result finished ~A" name-space (goal-status-to-string (send self :get-state)))
     (eq (send self :get-state) actionlib_msgs::GoalStatus::*succeeded*)))
  )
(In-package "USER")

(defun baxter-init ()
  (if (not (boundp '*ri*))
      (setq *ri* (instance baxter-interface :init)))
  (if (not (boundp '*baxter*))
      (setq *baxter* (instance baxter-robot :init)))
  )
