#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; This test is mainly copied from
;; https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_fetch_robot/fetcheus/test/test-fetcheus.l

(load "package://mycoboteus/mycobot-interface.l")

(deftest mycobot-kinematic-simulator-test
  (let ((ri (instance mycobot-interface :init))
        (mycobot (mycobot))
        diff-av)
    ;;
    (format t "check reset-pose")
    (send mycobot :reset-pose)
    (send ri :robot :reset-pose)
    (send ri :angle-vector (send mycobot :angle-vector))
    (send ri :wait-interpolation)
    ;; do not care 360 rotaiton
    (setq diff-av (send ri :sub-angle-vector (send ri :state :potentio-vector) (send mycobot :angle-vector)))
    (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send ri :robot :rarm :joint6 :joint-angle) (send mycobot :rarm :joint6 :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":reset-pose, diff-av ~A" diff-av))

    (format t "check :ik #f(280 0 100)")
    (send mycobot :rarm :inverse-kinematics (make-coords :pos #f(280 0 100)))
    (send ri :angle-vector (send mycobot :angle-vector))
    (send ri :wait-interpolation)
    (setq diff-av (send ri :sub-angle-vector (send ri :state :potentio-vector) (send mycobot :angle-vector)))
    (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send ri :robot :rarm :joint6 :joint-angle) (send mycobot :rarm :joint6 :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":ik 300 0 200, diff-av ~A" diff-av))
    ))

(deftest mycobot-init-test
  (let ()
    (mycobot-init)))

(run-all-tests)
(exit)
