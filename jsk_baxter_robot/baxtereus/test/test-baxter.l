#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(require "package://baxtereus/baxter-util.l")

(init-unit-test)

(deftest test-safe-pose
  (let (robot)
    (setq robot (instance baxter-robot-safe :init))
    (send robot :reset-pose)
    (assert (null (send robot :self-collision-check)))
    (send robot :reset-manip-pose)
    (assert (null (send robot :self-collision-check)))
    (send robot :tuck-pose)
    (assert (null (send robot :self-collision-check)))
    (send robot :untuck-pose)
    (assert (null (send robot :self-collision-check)))
    ))

(deftest test-unsafe-pose
  (let (robot)
    (setq robot (instance baxter-robot-safe :init))
    (send robot :angle-vector #f(0.0 74.2987 -79.5074 -174.983 146.163 -63.5022 -67.4432 39.1892 -20.0 -25.0 40.0 60.0 20.0 80.0 0.0))
    (assert (send robot :self-collision-check))
    ))

(run-all-tests)
(exit)
