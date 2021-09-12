#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")

(load "package://fetcheus/test/test-fetch-navigation-utils.l")

(init-unit-test)

(deftest test-move-to
  (let ((x 0.3) (y 0.3) (d 1.57) ;; [m] [m] [rad]
        pos0)
    (ros::ros-info "send :move-to ~A ~A ~A" x y d)
    (send *ri* :move-to
          (make-coords :pos (float-vector (* 1000 x) (* 1000 y) 0)
                       :rpy (float-vector d 0 0))
          :frame-id "map")
    (unix:sleep 1)
    (setq pos0 (send *ri* :state :worldcoords "map"))
    (ros::ros-info "reached to the goal")
    (ros::ros-info " difference-position: ~A -> ~A ~A" (send pos0 :worldpos) (- (elt (send pos0 :pos) 0) (* 1000 x)) (- (elt (send pos0 :pos) 1) (* 1000 y)))
    (ros::ros-info " difference-rotation: ~A -> ~A" (send pos0 :worldrot) (shortest-angle d (elt (car (send pos0 :rpy-angle)) 0)))
    ;; diff in 10cm
    (assert (< (abs (- (elt (send pos0 :pos) 0) (* 1000 x))) 100)
            (format nil "move-to moves incorrectly in x axis: ~A != ~A"
                    (* 1000 x) (elt (send pos0 :pos) 0)))
    (assert (< (abs (- (elt (send pos0 :pos) 1) (* 1000 y))) 100)
            (format nil "move-to moves incorrectly in y axis: ~A != ~A"
                    (* 1000 y) (elt (send pos0 :pos) 1)))
    (assert (< (abs (shortest-angle d (elt (car (send pos0 :rpy-angle)) 0))) 0.1)
            (format nil "go-pos-unsafe moves incorrectly in d axis: ~A != ~A"
                    d (elt (car (send pos0 :rpy-angle)) 0)))
    ))

(run-all-tests)
(exit)

