#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(require "package://spoteus/spot-utils.l")
(require "package://spoteus/spot-interface.l")

(init-unit-test)

(deftest test-pose
  (let (robot)
    (setq robot (instance spot-robot :init))
    ;; move robot arm to #f(600 0 600) relative to body
    (send robot :arm :inverse-kinematics (send (make-coords :pos #f(600 0 600)) :transform robot))
    ))

(defun look-at (robot)
  (let ((targets (list (make-cube 100 100 100 :pos #f(2000 0 0))
		       (make-cube 100 100 100 :pos #f(2000 1000 2000))
		       (make-cube 100 100 100 :pos #f(2000 -1000 1000))
		       (make-cube 100 100 100 :pos #f(0 2000 2000))
		       (make-cube 100 100 100 :pos #f(0 -2000 1000)))))
    (objects (append (list robot) targets))
    (dolist (target targets)
      (send robot :head :look-at (send target :worldpos) :debug-view :no-mesage)
      (send *irtviewer* :draw-objects))))

(deftest test-spot-look-at
  (let (robot)
    (setq robot (instance spot-robot :init))
    (look-at robot)
    (send robot :arm :inverse-kinematics (send (make-coords :pos #f(600 0 600)) :transform robot))
    (look-at robot)))

;; spot-interface is not work on simulator
(deftest test-spot-init
  (spot-init))

(run-all-tests)
(exit)
