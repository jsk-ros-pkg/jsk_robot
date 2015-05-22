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

(setq *pod-bin* '(#f(950.0 280.0 1695.0) #f(950.0 -2.449294e-14 1695.0) #f(950.0 -280.0 1695.0) #f(950.0 280.0 1465.0) #f(950.0 -2.449294e-14 1465.0) #f(950.0 -280.0 1465.0) #f(950.0 280.0 1235.0) #f(950.0 -2.449294e-14 1235.0) #f(950.0 -280.0 1235.0) #f(950.0 280.0 975.0) #f(950.0 -2.449294e-14 975.0) #f(950.0 -280.0 975.0)))

(deftest test-robot-ik-bin
  (let (robot)
    (setq robot (instance baxter-robot :init))
    (send robot :locate #f(0 0 950) :world)
    (objects (list robot))
    (dolist (pos *pod-bin*)
      (dolist (arm '(:larm :rarm))
        (assert (send robot arm :inverse-kinematics
                      (make-cascoords :pos (v+ pos #f(-90 0 0)))
                      :rotation-axis t
                      :debug-view t) "fail to solve ik")
        )) ;; do list
    )) ;; let

(deftest test-robot-ik-safe-bin
  (let (robot)
    (setq robot (instance baxter-robot-safe :init))
    (send robot :locate #f(0 0 950) :world)
    (objects (list robot))
    (dolist (pos *pod-bin*)
      (dolist (arm '(:larm :rarm))
        (send robot :reset-pose)
        (assert (send robot arm :inverse-kinematics
                      (make-cascoords :pos (v+ pos #f(-90 0 0)))
                      :rotation-axis t
                      :debug-view t) "fail to solve ik")
        )) ;; do list
    )) ;; let

(require "package://baxtereus/baxter-interface.l")
(deftest test-baxter-interface
  (let ()
    (setq *ri* (instance baxter-interface :init))
    ))


(run-all-tests)
(exit)
