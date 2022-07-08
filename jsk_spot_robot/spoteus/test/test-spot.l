#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(require "package://spoteus/spot-util.l")

(init-unit-test)

(deftest test-pose
  (let (robot)
    (setq robot (instance spot-robot :init))
    ))

(deftest test-spot-init
  (spot-init))

(run-all-tests)
(exit)
