#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "package://fetcheus/fetch-interface.l")

(init-unit-test)

(deftest instantiate-fetch
  (setq *fetch* (fetch)))

;; https://github.com/jsk-ros-pkg/jsk_robot/issues/702
(deftest ik-arm-torso
  (let ((robot (fetch)) waist-z-value)
    (setq waist-z-value (send robot :torso :waist-z :joint-angle))
    (objects (list robot))
    (assert (send robot :rarm :inverse-kinematics (make-coords :pos #f(800 0 1200)) :debug-view t))
    (assert (eps= waist-z-value
                  (send robot :torso :waist-z :joint-angle) 10)
            (format nil "check if torso did not move ~A vs ~A" waist-z-value
                    (send robot :torso :waist-z :joint-angle)))

    ;;(setq waist-z-value (send robot :torso :waist-z :joint-angle))
    (assert (send robot :inverse-kinematics (make-coords :pos #f(800 0 1300)) :debug-view t))
    (assert (not (eps= waist-z-value
                       (send robot :torso :waist-z :joint-angle) 10))
            (format nil "check if torso moved ~A vs ~A" waist-z-value
                    (send robot :torso :waist-z :joint-angle)))

    (setq waist-z-value (send robot :torso :waist-z :joint-angle))
    (assert (send robot :inverse-kinematics (make-coords :pos #f(800 0 1200)) :use-torso nil :debug-view t))
    (assert (eps= waist-z-value
                  (send robot :torso :waist-z :joint-angle) 10)
            (format nil "check if torso did not move ~A vs ~A" waist-z-value
                    (send robot :torso :waist-z :joint-angle)))
    ))


(deftest instantiate-fetch-interface
  (setq *ri* (instance fetch-interface :init)))

(run-all-tests)
(exit)

