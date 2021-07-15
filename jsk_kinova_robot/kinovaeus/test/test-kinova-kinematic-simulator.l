#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; This test is mainly copied from
;; https://github.com/jsk-ros-pkg/jsk_robot/blob/master/jsk_fetch_robot/fetcheus/test/test-fetcheus.l

(load "package://kinovaeus/kinova-interface.l")

(deftest kinova-kinematic-simulator-test
  (dolist (robot-type (list :gen3_robotiq_2f_85 :gen3_robotiq_2f_140 :gen3_lite_gen3_lite_2f))
    (let ((ri (instance kinova-interface :init :type robot-type))
          (kinova (kinova :type robot-type))
          diff-av)
      ;;
      (format t "check reset-pose")
      (send kinova :reset-pose)
      (send ri :robot :reset-pose)
      (send ri :angle-vector (send kinova :angle-vector))
      (send ri :wait-interpolation)
      ;; do not care 360 rotaiton
      (setq diff-av (send ri :sub-angle-vector (send ri :state :potentio-vector) (send kinova :angle-vector)))
      (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send ri :robot :rarm :wrist-y :joint-angle) (send kinova :rarm :wrist-y :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
      (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":reset-pose, diff-av ~A" diff-av))

      (format t "check :ik #f(300 0 200), do not care about 360 degree rotation")
      (send kinova :arm :inverse-kinematics (make-coords :pos #f(300 0 200) :rpy (float-vector 0 pi/2 0)))
      (send ri :angle-vector (send kinova :angle-vector))
      (send ri :wait-interpolation)
      ;; do not care 360 rotaiton
      (setq diff-av (send ri :sub-angle-vector (send ri :state :potentio-vector) (send kinova :angle-vector)))
      (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send ri :robot :rarm :wrist-y :joint-angle) (send kinova :rarm :wrist-y :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
      (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":ik 300 0 200, diff-av ~A" diff-av))

      ;; do care 360 rotation
      (format t "check :init-pose")
      (send kinova :init-pose) ;; reset
      (send ri :robot :init-pose)
      (send ri :angle-vector (send kinova :angle-vector))
      (send ri :wait-interpolation)
      (setq diff-av (v- (send ri :state :potentio-vector) (send kinova :angle-vector)))
      (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send ri :robot :rarm :wrist-y :joint-angle) (send kinova :rarm :wrist-y :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
      (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":init-pose, diff-av ~A" diff-av))

      ;; 150 is 150
      (format t "check :joint-angle 150, this should goes to 150")
      (send kinova :rarm :wrist-y :joint-angle 150)
      (send ri :angle-vector (send kinova :angle-vector))
      (send ri :wait-interpolation)
      (setq diff-av (v- (send ri :state :potentio-vector) (send kinova :angle-vector)))
      (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send ri :robot :rarm :wrist-y :joint-angle) (send kinova :rarm :wrist-y :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
      (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":joint-angle 150, diff-av ~A" diff-av))

      (format t "check :joint-angle-sequence 150 0 -150, this should goes to -150")
      ;; to send to -150, use :angle-vector-sequence
      (send ri :angle-vector-sequence (list
                                       (progn
                                         (send kinova :rarm :wrist-y :joint-angle  150)
                                         (send kinova :angle-vector))
                                       (progn
                                         (send kinova :rarm :wrist-y :joint-angle   0)
                                         (send kinova :angle-vector))
                                       (progn
                                         (send kinova :rarm :wrist-y :joint-angle -150)
                                         (send kinova :angle-vector))))
      (send ri :wait-interpolation)
      (setq diff-av (v- (send ri :state :potentio-vector) (send kinova :angle-vector)))
      (format t ":state ~A, :robot ~A, diff ~A, ~A~%" (send ri :robot :rarm :wrist-y :joint-angle) (send kinova :rarm :wrist-y :joint-angle)  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
      (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":angle-vector-sequence, diff-av ~A" diff-av))
      )))

(deftest kinova-init-test
  (let ()
    (kinova-init :type :gen3_robotiq_2f_85)
    (kinova-init :type :gen3_robotiq_2f_140)
    (kinova-init :type :gen3_lite_gen3_lite_2f)))

(run-all-tests)
(exit)
