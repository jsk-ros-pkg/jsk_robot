#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(require "package://kinovaeus/kinova.l")
;; (require "package://kinovaeus/kinova-interface.l")

(init-unit-test)

(deftest test-pose
  (let (robot)
    (setq robot (kinova))
    (objects (list robot))
    ;; rotation-angle NIL is returned when rotation-matrix is a unit-matrix
    (warning-message 2 "~A end-corods ~A~%" (send robot :name) (rotation-angle (send robot :arm :end-coords :worldrot)))
    (assert (eps= (norm (send (make-coords) :difference-rotation (send robot :arm :end-coords))) 0))

    (setq robot (kinova :type :gen3_robotiq_2f_140))
    (objects (list robot))
    (warning-message 2 "~A end-corods ~A~%" (send robot :name) (rotation-angle (send robot :arm :end-coords :worldrot)))
    (assert (eps= (norm (send (make-coords) :difference-rotation (send robot :arm :end-coords))) 0))

    (setq robot (kinova :type :gen3_lite_gen3_lite_2f))
    (objects (list robot))
    (warning-message 2 "~A end-corods ~A~%" (send robot :name) (rotation-angle (send robot :arm :end-coords :worldrot)))
    (assert (eps= (norm (send (make-coords) :difference-rotation (send robot :arm :end-coords))) 0))
    ))


(run-all-tests)
(exit)

