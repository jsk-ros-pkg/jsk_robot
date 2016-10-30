;; test-base-trajectory-logger.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "pr2_controllers_msgs")
(ros::load-ros-manifest "posedetection_msgs")

(init-unit-test)

(deftest test-base-trajectory-logger ()
  ;; wait till mongo server is ready
  (mongo::wait-for-server)

  (setq start-time-ros (ros::time-now))
  (setq start-time (now))

  ;; send cmd_vel vx=+0.55m/s for 5 seconds
  (ros::rate 10)
  (setq cmd-msg (instance geometry_msgs::Twist :init))
  (send cmd-msg :linear :x 0.55)
  (ros::advertise "/base_controller/command" geometry_msgs::Twist 1)
  (unix:sleep 1)
  (while (< (send (ros::time- (ros::time-now) start-time-ros) :to-sec) 5.0)
    (ros::publish "/base_controller/command" cmd-msg)
    (ros::sleep))

  ;; check recorded log data
  (setq res (mongo::query geometry_msgs::TransformStamped
                          :msg-only t
                          :meta `((:inserted_at .
                                   ((:gt . ,start-time))))))
  (ros::ros-info "~A messages are logged in database" (length res))
  (dolist (msg res)
    (print-ros-msg msg))
  (assert (> (length res) 0)
          "base trajectory is logged during moving"))

(deftest test-action-logger ()
  ;; wait till mongo server is ready
  (mongo::wait-for-server)

  (setq start-time (now))

  ;; send gripper command
  (setq *ri* (instance pr2-interface :init))
  (send *ri* :stop-grasp :rarm :wait t)
  (send *ri* :start-grasp :rarm)

  ;; check recorded log data
  (setq res (mongo::query pr2_controllers_msgs::Pr2GripperCommandActionResult
                          :msg-only t
                          :meta `((:inserted_at .
                                   ((:gt . ,start-time))))))
  (ros::ros-info "~A result messages are logged in database" (length res))
  (dolist (msg res)
    (print-ros-msg msg))
  (assert (> (length res) 0)
          "action results are recorded")

  (setq res (mongo::query pr2_controllers_msgs::Pr2GripperCommandActionGoal
                          :msg-only t
                          :meta `((:inserted_at .
                                   ((:gt . ,start-time))))))
  (ros::ros-info "~A goal messages are logged in database" (length res))
  (dolist (msg res)
    (print-ros-msg msg))
  (assert (> (length res) 0)
          "action goals are recorded")
  (setq res (mongo::query pr2_controllers_msgs::Pr2GripperCommandActionFeedback
                          :msg-only t
                          :meta `((:inserted_at .
                                   ((:gt . ,start-time))))))
  (ros::ros-info "~A feedback messages are logged in database" (length res))
  (dolist (msg res)
    (print-ros-msg msg))
  (assert (> (length res) 0)
          "action feedbacks are recorded"))

(deftest test-object-detection-logger ()
  ;; wait till mongo server is ready
  (mongo::wait-for-server)

  (setq start-time (now))

  ;; publish dummy object detection msgs
  (setq od-msg (instance posedetection_msgs::ObjectDetection :init))
  (send od-msg :header :frame_id "head_mount_kinect_rgb_optical_frame")
  (setq o6d-msg (instance posedetection_msgs::Object6DPose :init :type "dummy"))
  (push o6d-msg od-msg)
  (ros::advertise "/kinect_head_c2/depth_registered/ObjectDetection"
                  posedetection_msgs::ObjectDetection)
  (unix:sleep 1)
  (ros::publish "/kinect_head_c2/depth_registered/ObjectDetection" od-msg)

  (setq res (mongo::query geometry_msgs::TransformStamped
                          :msg-only t
                          :query '((:header.frame_id . "head_mount_kinect_rgb_optical_frame"))
                          :meta `((:inserted_at .
                                   (:gt . ,start-time)))))
  (ros::ros-info "~A messages are logged in database" (length res))
  (dolist (msg res)
    (print-ros-msg msg))
  (assert (> (length res) 0)
          "action results are recorded"))

(run-all-tests)
(exit)
