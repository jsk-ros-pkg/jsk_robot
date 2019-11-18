#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "package://baxtereus/baxter-interface.l")

(ros::load-ros-manifest "baxtereus")
(ros::roseus "baxter-moveit")

;; add ros::duration-sleep
;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/459
(defun sleep-rostime (n)
  (let ((tm0 (ros::time-now)) (tm1 (ros::time-now)))
    (while (<= (send (ros::time- tm1 tm0) :to-sec) n)
      (unix::sleep 1)
      (setq tm1 (ros::time-now)))))

(deftest test-baxter-moveit-init ()
  (let ((waiting-count 0))
    (while
      (null (and (one-shot-subscribe "/clock" rosgraph_msgs::Clock :timeout 3000)
                 (one-shot-subscribe "/robot/state" baxter_core_msgs::AssemblyState :timeout 3000)))
      (ros::ros-info "waiting gazebo")
      (setq waiting-count (incf waiting-count))
      (assert (> 300 waiting-count) "gazebo did not start.")
      (sleep-rostime 3))
    (ros::ros-info "gazebo ready")

    (setq waiting-count 0)
    (while
      (null (one-shot-subscribe "/move_group/status" actionlib_msgs::GoalStatusArray :timeout 3000))
      (ros::ros-info "waiting moveit")
      (setq waiting-count (incf waiting-count))
      (assert (> 300 waiting-count) "moveit did not start.")
      (sleep-rostime 3))
    (ros::ros-info "moveit ready")

    (setq *ri* (instance baxter-interface :init))
    (setq *baxter* (instance baxter-robot :init))
    (assert (not (send *ri* :simulation-modep)) "baxter-interface is in simulation mode")
    (send *baxter* :reset-pose)
    (send *baxter* :rarm :move-end-pos #f(50 0 0) :world)
    (send *baxter* :larm :move-end-pos #f(50 0 0) :world)
    (setq *reset-pose-mod* (send *baxter* :angle-vector))
    ))

(deftest test-baxter-moveit-angle-vector()
  (let (av)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector *reset-pose-mod*))
    (send *ri* :wait-interpolation)

    (setq av (send *ri* :angle-vector (send *baxter* :reset-pose) 800))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :reset-pose) 800)")

    (setq av (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) 800 :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) 800 :rarm-controller 0)")

    (setq av (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) 800 :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) 800 :larm-controller 0)")

    (setq av (send *ri* :angle-vector (send *baxter* :reset-pose) 800 :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :reset-pose) 800 :rarm-controller 0 :move-arm :rarm)")

    (setq av (send *ri* :angle-vector (send *baxter* :reset-pose) 800 :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :reset-pose) 800 :larm-controller 0 :move-arm :larm)")

    (setq av (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) 800 nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) 800 nil 0 :move-arm :arms)")
    ))

(deftest test-baxter-moveit-angle-vector-fast()
  (let (av)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector *reset-pose-mod*))
    (send *ri* :wait-interpolation)

    (setq av (send *ri* :angle-vector (send *baxter* :reset-pose) :fast))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :reset-pose) :fast)")

    (setq av (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) :fast :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) :fast :rarm-controller 0)")

    (setq av (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) :fast :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) :fast :larm-controller 0)")

    (setq av (send *ri* :angle-vector (send *baxter* :reset-pose) :fast :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :reset-pose) :fast :rarm-controller 0 :move-arm :rarm)")

    (setq av (send *ri* :angle-vector (send *baxter* :reset-pose) :fast :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :reset-pose) :fast :larm-controller 0 :move-arm :larm)")

    (setq av (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) :fast nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :angle-vector *reset-pose-mod*) :fast nil 0 :move-arm :arms)")
    ))


(deftest test-baxter-moveit-angle-vector-sequence ()
  (let (avs)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector *reset-pose-mod*))
    (send *ri* :wait-interpolation)

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 :rarm-controller 0 :move-arm :rarm)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 :larm-controller 0 :move-arm :larm)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) 1600 nil 0 :move-arm :arms)")
    ))

(deftest test-baxter-moveit-angle-vector-sequence-fast ()
  (let (avs)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector *reset-pose-mod*))
    (send *ri* :wait-interpolation)

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :reset-pose) (send *baxter* :angle-vector *reset-pose-mod*)) :fast nil 0 :move-arm :arms)")
    ))

(deftest test-baxter-moveit-angle-vector-start-time ()
  (let (tm-0 tm-1 tm-diff)
    (send *ri* :angle-vector (send *baxter* :reset-pose))
    (send *ri* :wait-interpolation)
    (send *baxter* :rarm :move-end-pos #f(50 0 0) :world)
    (send *ri* :angle-vector (send *baxter* :angle-vector) 1000 nil 3)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 3) (format nil "start-time is ignored. Traj finishes at ~A" tm-diff))
    (assert (< tm-diff 5) (format nil "start-time is considered multiple times. Traj finishes at ~A" tm-diff))
    (send *ri* :angle-vector (send *baxter* :reset-pose) 1000 nil 3 :start-offset-time 0)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (< tm-diff 3) (format nil "start-offset-time cannot overwrite start-time. Traj finishes at ~A" tm-diff))
    ))

(deftest test-baxter-moveit-angle-vector-sequence-start-time ()
  (let (avs tm-0 tm-1 tm-diff)
    (send *ri* :angle-vector (send *baxter* :reset-pose))
    (send *ri* :wait-interpolation)
    (push (send *baxter* :rarm :move-end-pos #f(50 0 0) :world) avs)
    (push (send *baxter* :rarm :move-end-pos #f(50 0 0) :world) avs)
    (setq avs (reverse avs))
    (send *ri* :angle-vector-sequence avs 1000 nil 3)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (> tm-diff 3) (format nil "start-time is ignored. Traj finishes at ~A" tm-diff))
    (assert (< tm-diff 6) (format nil "start-time is considered multiple times. Traj finishes at ~A" tm-diff))
    (send *ri* :angle-vector-sequence avs 1000 nil 3 :start-offset-time 0)
    (setq tm-0 (ros::time-now))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (setq tm-diff (send (ros::time- tm-1 tm-0) :to-sec))
    (ros::ros-info "time for duration ~A" tm-diff)
    (assert (< tm-diff 3) (format nil "start-offset-time cannot overwrite start-time. Traj finishes at ~A" tm-diff))
    ))

(run-all-tests)
(exit)
