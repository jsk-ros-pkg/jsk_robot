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
    (format t "check reset-pose")
    (send *fetch* :reset-pose)
    (send *ri* :robot :reset-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; do not care 360 rotaiton
    (setq diff-av (send *ri* :sub-angle-vector (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))
    (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send *ri* :robot :rarm :wrist-r :joint-angle) (send *fetch* :rarm :wrist-r :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":reset-pose, diff-av ~A" diff-av))

    (format t "check :ik #f(800 0 1300), do not care about 360 degree rotation")
    (send *fetch* :inverse-kinematics (make-coords :pos #f(800 0 1300)))
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (send *ri* :wait-interpolation)
    ;; do not care 360 rotaiton
    (setq diff-av (send *ri* :sub-angle-vector (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))
    (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send *ri* :robot :rarm :wrist-r :joint-angle) (send *fetch* :rarm :wrist-r :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":ik 800 0 1300, diff-av ~A" diff-av))

    ;; do care 360 rotation
    (format t "check :init-pose")
    (send *fetch* :init-pose) ;; reset
    (send *ri* :robot :init-pose)
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (send *ri* :wait-interpolation)
    (setq diff-av (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))
    (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send *ri* :robot :rarm :wrist-r :joint-angle) (send *fetch* :rarm :wrist-r :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":init-pose, diff-av ~A" diff-av))

    ;; 150 is 150
    (format t "check :joint-angle 150, this should goes to 150")
    (send *fetch* :rarm :wrist-r :joint-angle 150)
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (send *ri* :wait-interpolation)
    (setq diff-av (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))
    (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send *ri* :robot :rarm :wrist-r :joint-angle) (send *fetch* :rarm :wrist-r :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":joint-angle 150, diff-av ~A" diff-av))

    ;; from 150 to -150, robot moves to 210 by :angle-vector
    (format t "check :joint-angle -150, this should goes to 210, by selecting minimum motion")
    (send *fetch* :rarm :wrist-r :joint-angle -150)
    (send *ri* :angle-vector (send *fetch* :angle-vector))
    (send *ri* :wait-interpolation)
    (setq diff-av (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))
    (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send *ri* :robot :rarm :wrist-r :joint-angle) (send *fetch* :rarm :wrist-r :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (null (eps= (norm diff-av) 0.0 10.0)) (format nil ":joint-angle -150, diff-av ~A" diff-av))

    (format t "check :joint-angle-sequence 150 0 -150, this should goes to -150")
    ;; to send to -150, use :angle-vector-sequence
    (send *ri* :angle-vector-sequence (list
                                       (progn
                                         (send *fetch* :rarm :wrist-r :joint-angle  150)
                                         (send *fetch* :angle-vector))
                                       (progn
                                         (send *fetch* :rarm :wrist-r :joint-angle   0)
                                         (send *fetch* :angle-vector))
                                       (progn
                                         (send *fetch* :rarm :wrist-r :joint-angle -150)
                                         (send *fetch* :angle-vector))))
    (send *ri* :wait-interpolation)
    (setq diff-av (v- (send *ri* :state :potentio-vector) (send *fetch* :angle-vector)))
    (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send *ri* :robot :rarm :wrist-r :joint-angle) (send *fetch* :rarm :wrist-r :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":angle-vector-sequence, diff-av ~A" diff-av))
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

(deftest fetch-go-velocity
  (let (go-vel-arg)
    (setq go-vel-arg (list 0.1 0 0.5))
    (assert (send* *ri* :go-velocity go-vel-arg)
            (format nil ":go-velocity doesn't return t with argument ~A" go-vel-arg))
    (setq go-vel-arg (list 0.1 0.1 0.5))
    (assert (not (send* *ri* :go-velocity go-vel-arg))
            (format nil ":go-velocity returns t with argument ~A" go-vel-arg))
    ))

(deftest fetch-use-torso
  (let ()
    (setq *ri* (instance fetch-interface :init))
    (assert (send *ri* :angle-vector (send *fetch* :angle-vector) 3000 :use-torso t)
            "failed to do :angle-vector with :use-torso in kinematics simulator")
    (assert (send *ri* :angle-vector-sequence
                  (list (send *fetch* :init-pose) (send *fetch* :angle-vector))
                  (list 3000 3000) :use-torso t)
            "failed to do :angle-vector-sequence with :use-torso in kinematics simulator")
    ))

(deftest fetch-init-test
  (let ()
    (fetch-init)))

(run-all-tests)
(exit)

