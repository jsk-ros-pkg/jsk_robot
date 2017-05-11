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
  (let (diff-av)
    (setq *ri* (instance fetch-interface :init))
    ;;
    (send *fetch* :reset-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (setq diff-av (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))
    (assert (eps= (norm diff-av) 0.0 5.0) (format nil "diff-av ~A" diff-av))

    (send *fetch* :inverse-kinematics (make-coords :pos #f(800 0 1300)))
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (setq diff-av (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))
    (assert (eps= (norm diff-av) 0.0 5.0) (format nil "diff-av ~A" diff-av))
    ))

;; https://github.com/jsk-ros-pkg/jsk_robot/pull/771/files
(deftest fetch-head-look-at
  (let ((robot (fetch)))
    (send robot :init-pose)
    (send robot :rarm :inverse-kinematics (make-coords :pos #f(700 350 700)))
    (send robot :look-at-hand :rarm)

    (warning-message 2 "head angle-vector ~A" (send robot :head :angle-vector))
    (assert (v< #f(25 25) (send robot :head :angle-vector)))
    (assert (v< (send robot :head :angle-vector) #f(30 30)))
    ))


(run-all-tests)
(exit)

