#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "package://sciurus17eus/sciurus17-interface.l")

(ros::load-ros-manifest "sciurus17eus")
(ros::roseus "sciurus17-moveit")

;; add ros::duration-sleep
;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/459
(defun sleep-rostime (n)
  (let ((tm0 (ros::time-now)) (tm1 (ros::time-now)))
    (while (<= (send (ros::time- tm1 tm0) :to-sec) n)
      (unix::sleep 1)
      (setq tm1 (ros::time-now)))))

(deftest test-sciurus17-moveit-init ()
  (let ((waiting-count 0) (js nil))
    ;; Wait for robot to be ready
    (ros::ros-info "waiting for robot connection")
    (unix::sleep 5)  ;; Give some time for robot to initialize

    (setq waiting-count 0)
    (while
      (progn
        (setq js (one-shot-subscribe "/joint_states" sensor_msgs::JointState :timeout 3000))
        (or (null js)
            (< (length (send js :position)) 10)))  ;; wait until all joints are available
      (ros::ros-info "waiting joint_states (robot connection and initialization)")
      (setq waiting-count (incf waiting-count))
      (assert (> 300 waiting-count) "robot did not connect or initialize.")
      (unix::sleep 3))
    (ros::ros-info "robot connected and initialized")

    ;; wait additional time for controllers to be ready
    (ros::ros-info "waiting for controllers to be ready")
    (unix::sleep 5)

    (setq waiting-count 0)
    (while
      (null (one-shot-subscribe "/move_group/status" actionlib_msgs::GoalStatusArray :timeout 3000))
      (ros::ros-info "waiting moveit")
      (setq waiting-count (incf waiting-count))
      (assert (> 300 waiting-count) "moveit did not start.")
      (unix::sleep 3))
    (ros::ros-info "moveit ready")

    (sciurus17-init :moveit t)
    (assert *ri* "sciurus17-interface initialized")
    (assert *sciurus17* "sciurus17-robot initialized")
    (send *sciurus17* :init-pose)
    (send *sciurus17* :rarm :move-end-pos #f(30 0 0) :world)
    (send *sciurus17* :larm :move-end-pos #f(30 0 0) :world)
    (setq *init-pose-mod* (send *sciurus17* :angle-vector))
    ))

(deftest test-sciurus17-moveit-angle-vector()
  (let (av)
    (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector *init-pose-mod*))
    (send *ri* :wait-interpolation)

    (setq av (send *ri* :angle-vector (send *sciurus17* :init-pose) 800))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :init-pose) 800)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) 800 :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) 800 :rarm-controller 0)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) 800 :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) 800 :larm-controller 0)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :init-pose) 800 :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :init-pose) 800 :rarm-controller 0 :move-arm :rarm)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :init-pose) 800 :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :init-pose) 800 :larm-controller 0 :move-arm :larm)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) 800 nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) 800 nil 0 :move-arm :arms)")
    ))

(deftest test-sciurus17-moveit-angle-vector-fast()
  (let (av)
    (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector *init-pose-mod*))
    (send *ri* :wait-interpolation)

    (setq av (send *ri* :angle-vector (send *sciurus17* :init-pose) :fast))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :init-pose) :fast)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) :fast :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) :fast :rarm-controller 0)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) :fast :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) :fast :larm-controller 0)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :init-pose) :fast :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :init-pose) :fast :rarm-controller 0 :move-arm :rarm)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :init-pose) :fast :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :init-pose) :fast :larm-controller 0 :move-arm :larm)")

    (setq av (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) :fast nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *sciurus17* :angle-vector *init-pose-mod*) :fast nil 0 :move-arm :arms)")
    ))


(deftest test-sciurus17-moveit-angle-vector-sequence ()
  (let (avs)
    (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector *init-pose-mod*))
    (send *ri* :wait-interpolation)

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 :rarm-controller 0 :move-arm :rarm)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 :larm-controller 0 :move-arm :larm)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) 1600 nil 0 :move-arm :arms)")
    ))

(deftest test-sciurus17-moveit-angle-vector-sequence-fast ()
  (let (avs)
    (send *ri* :angle-vector-raw (send *sciurus17* :angle-vector *init-pose-mod*))
    (send *ri* :wait-interpolation)

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *sciurus17* :init-pose) (send *sciurus17* :angle-vector *init-pose-mod*)) :fast nil 0 :move-arm :arms)")
    ))

(deftest test-sciurus17-moveit-angle-vector-start-time ()
  (let (tm-0 tm-1 tm-diff)
    (send *ri* :angle-vector (send *sciurus17* :init-pose))
    (send *ri* :wait-interpolation)
    (send *sciurus17* :rarm :move-end-pos #f(10 0 0) :world)
    (send *ri* :angle-vector (send *sciurus17* :angle-vector) 1000 nil 3)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 3) (format nil "start-time is ignored. Traj finishes at ~A" tm-diff))
    (assert (< tm-diff 6) (format nil "start-time is considered multiple times. Traj finishes at ~A" tm-diff))
    (send *ri* :angle-vector (send *sciurus17* :init-pose) 1000 nil 3 :start-offset-time 0)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (< tm-diff 6) (format nil "start-offset-time cannot overwrite start-time. Traj finishes at ~A" tm-diff))
    ))

(deftest test-sciurus17-moveit-angle-vector-sequence-start-time ()
  (let (avs tm-0 tm-1 tm-diff)
    (send *ri* :angle-vector (send *sciurus17* :init-pose))
    (send *ri* :wait-interpolation)
    (push (send *sciurus17* :rarm :move-end-pos #f(10 0 0) :world) avs)
    (push (send *sciurus17* :rarm :move-end-pos #f(10 0 0) :world) avs)
    (setq avs (reverse avs))
    (send *ri* :angle-vector-sequence avs 1000 nil 3)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 3) (format nil "start-time is ignored. Traj finishes at ~A" tm-diff))
    (assert (< tm-diff 8) (format nil "start-time is considered multiple times. Traj finishes at ~A" tm-diff))
    (send *ri* :angle-vector-sequence avs 1000 nil 3 :start-offset-time 0)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (< tm-diff 6) (format nil "start-offset-time cannot overwrite start-time. Traj finishes at ~A" tm-diff))
    ))

(run-all-tests)
(exit)
