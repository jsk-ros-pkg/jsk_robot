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
    (setq *baxter* (instance baxter-robot :init))))

(deftest test-baxter-moveit-angle-vector()
  (let (av)
    (send *ri* :angle-vector-raw (send *baxter* :reset-pose))
    (send *ri* :wait-interpolation)

    (setq av (send *ri* :angle-vector (send *baxter* :init-pose) 3000))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :init-pose) 3000)")

    (setq av (send *ri* :angle-vector (send *baxter* :untuck-pose) 3000 :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :untuck-pose) 3000 :rarm-controller 0)")

    (setq av (send *ri* :angle-vector (send *baxter* :untuck-pose) 3000 :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :untuck-pose) 3000 :larm-controller 0)")

    (setq av (send *ri* :angle-vector (send *baxter* :init-pose) 3000 :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :init-pose) 3000 :rarm-controller 0 :move-arm :rarm)")

    (setq av (send *ri* :angle-vector (send *baxter* :init-pose) 3000 :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :init-pose) 3000 :larm-controller 0 :move-arm :larm)")

    (setq av (send *ri* :angle-vector (send *baxter* :untuck-pose) 3000 nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :untuck-pose) 3000 nil 0 :move-arm :arms)")
    ))

(deftest test-baxter-moveit-angle-vector-fast()
  (let (av)
    (send *ri* :angle-vector-raw (send *baxter* :reset-pose))
    (send *ri* :wait-interpolation)

    (setq av (send *ri* :angle-vector (send *baxter* :init-pose) :fast))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :init-pose) :fast)")

    (setq av (send *ri* :angle-vector (send *baxter* :untuck-pose) :fast :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :untuck-pose) :fast :rarm-controller 0)")

    (setq av (send *ri* :angle-vector (send *baxter* :untuck-pose) :fast :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :untuck-pose) :fast :larm-controller 0)")

    (setq av (send *ri* :angle-vector (send *baxter* :init-pose) :fast :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :init-pose) :fast :rarm-controller 0 :move-arm :rarm)")

    (setq av (send *ri* :angle-vector (send *baxter* :init-pose) :fast :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :init-pose) :fast :larm-controller 0 :move-arm :larm)")

    (setq av (send *ri* :angle-vector (send *baxter* :untuck-pose) :fast nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert av "failed: (send *ri* :angle-vector (send *baxter* :untuck-pose) :fast nil 0 :move-arm :arms)")
    ))


(deftest test-baxter-moveit-angle-vector-sequence ()
  (let (avs)
    (send *ri* :angle-vector-raw (send *baxter* :reset-pose))
    (send *ri* :wait-interpolation)

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) 3000 nil 0 :move-arm :arms)")
    ))

(deftest test-baxter-moveit-angle-vector-sequence-fast ()
  (let (avs)
    (send *ri* :angle-vector-raw (send *baxter* :reset-pose))
    (send *ri* :wait-interpolation)

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast :rarm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast :larm-controller 0))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast :rarm-controller 0 :move-arm :rarm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast :rarm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast :larm-controller 0 :move-arm :larm))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast :larm-controller 0)")

    (setq avs (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast nil 0 :move-arm :arms))
    (send *ri* :wait-interpolation)
    (assert avs "failed: (send *ri* :angle-vector-sequence (list (send *baxter* :init-pose) (send *baxter* :untuck-pose)) :fast nil 0 :move-arm :arms)")
    ))

(run-all-tests)
(exit)
