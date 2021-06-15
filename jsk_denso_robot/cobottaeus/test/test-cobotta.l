#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(require "package://cobottaeus/cobotta-utils.l")
(require "package://cobottaeus/cobotta-interface.l")

(init-unit-test)

(deftest test-pose
  (let (robot)
    (setq robot (instance cobotta-robot :init))
    ))

(deftest test-cobotta-move
  (cobotta-init)
  (send *ri* :clear-error)

  (send *ri* :stop-grasp)
  (send *cobotta* :reset-pose)
  (assert (send *ri* :angle-vector (send *cobotta* :angle-vector)))

  (send *cobotta* :angle-vector #f(90.0 0.0 25.0 0.0 0.0 0.0))
  (assert (send *ri* :angle-vector (send *cobotta* :angle-vector)))

  (send *cobotta* :arm :inverse-kinematics (make-coords :pos #f(200 0 100) :rpy (float-vector 0 pi 0)))
  (assert (send *ri* :angle-vector (send *cobotta* :angle-vector)))
  (send *ri* :start-grasp)

  (send *cobotta* :arm :inverse-kinematics (make-coords :pos #f(200 0 300) :rpy (float-vector 0 pi 0)))
  (assert (send *ri* :angle-vector (send *cobotta* :angle-vector)))
  )


(run-all-tests)
(cobotta-exit)

