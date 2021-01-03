#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(require "package://kinovaeus/gen3-lite-utils.l")
(require "package://kinovaeus/gen3-lite-interface.l")

(init-unit-test)

(gen3-lite-init)

(deftest test-gen3-lite-move-moveit

  (send *gen3-lite* :reset-pose)
  (assert (send *ri* :angle-vector (send *gen3-lite* :angle-vector)))
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :wait t)

  (send *gen3-lite* :angle-vector #f(90.0 0.0 25.0 0.0 0.0 0.0 0.0))
  (assert (send *ri* :angle-vector-raw (send *gen3-lite* :angle-vector)))
  (send *ri* :wait-interpolation)

  (send *gen3-lite* :arm :inverse-kinematics (make-coords :pos #f(300 0 200) :rpy (float-vector pi  0 pi)) :debug-view t)
  (assert (send *ri* :angle-vector (send *gen3-lite* :angle-vector)))
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :wait t)
  )


(run-all-tests)
(exit)
