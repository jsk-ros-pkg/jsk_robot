#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "package://jsk_fetch_gazebo_demo/euslisp/demo.l")

(deftest init-and-move ()
  (let ((pos0 #f(2250 3118 0)) pos1)
    ;; Create *ri* to use fetch with MoveIt! and Navigation stack
    (fetch-init)

    ;; Move the base to be in front of the table
    ;; Demonstrates the use of the navigation stack
    (setq pos1 (send (send *ri* :state :worldcoords "map") :worldpos))
    (ros::ros-warn " goal position: ~A (diff:~A)" pos1 (norm (v- pos1 pos0)))
    (ros::ros-warn "Moving to table...")
    ;; first time
    (send *ri* :move-to (make-coords :pos pos0) :frame-id "map")
    (setq pos1 (send (send *ri* :state :worldcoords "map") :worldpos))
    (ros::ros-warn " goal position: ~A (diff:~A)" pos1 (norm (v- pos1 pos0)))
    (assert (< (norm  (v- pos1 pos0)) 500)
            (format nil "go-pos moves incorrectly : ~A" (norm  (v- pos0 pos1))))
    ;; second time
    (send *ri* :move-to (make-coords :pos pos0) :frame-id "map")
    (setq pos1 (send (send *ri* :state :worldcoords "map") :worldpos))
    (ros::ros-warn " goal position: ~A (diff:~A)" pos1 (norm (v- pos1 pos0)))
    (assert (< (norm  (v- pos1 pos0)) 500)
            (format nil "go-pos moves incorrectly : ~A" (norm  (v- pos0 pos1))))
    ))

(run-all-tests)
(exit)


