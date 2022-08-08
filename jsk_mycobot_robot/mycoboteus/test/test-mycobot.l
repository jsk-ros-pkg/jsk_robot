#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(require "package://mycoboteus/mycobot.l")

(init-unit-test)

(deftest test-end-coords-pose
  (let ((robot (mycobot)))
    (objects (list robot))
    ;; rotation-angle NIL is returned when rotation-matrix is a unit-matrix
    (warning-message 2 "~A end-corods ~A~%" (send robot :name) (rotation-angle (send robot :rarm :end-coords :worldrot)))
    (assert (eps= (norm (send (make-coords) :difference-rotation (send robot :rarm :end-coords))) 0))
    ))

(run-all-tests)
(exit)
