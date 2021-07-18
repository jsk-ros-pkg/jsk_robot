#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")

(load "package://fetcheus/test/test-fetch-navigation-utils.l")

(init-unit-test)

(deftest test-go-stop
  (let ((x 0.3) (y 0.3) (d 90)
        pos0 pos1)
    (ros::ros-info "send :go-pos ~A ~A ~A, assuming it takse longer than 2 sec" x y d)
    (send *ri* :go-pos-no-wait x y d)
    (send (instance ros::duration :init 2) :sleep)
    (setq pos0 (send *ri* :state :worldcoords "map"))
    (ros::ros-info "get current locateion ~A and send :go-stop" pos0)
    (send *ri* :go-stop)
    (unix:sleep 1)
    (assert (equal (send (*ri* . move-base-action) :get-state) actionlib_msgs::GoalStatus::*preempted*)) ;; make sure go-pos is preempted
    (setq pos1 (send *ri* :state :worldcoords "map"))
    (setq diff-pos (norm (send pos0 :difference-position pos1)))
    (ros::ros-info "get current locateion ~A, and diff position ~A" pos1 diff-pos)
    ;; diff in 3cm
    (assert (< diff-pos 30) (format nil "go-stop does not stop: ~A > 30" diff-pos))))

(defun go-pos-func (&key (x 0.3) (y 0.3) (d 90) (func :go-pos)) ;; [m] [m] [degree]
  (let (pos0 pos1 pos2 diff-pos diff-rot tm0 tm1)
    (setq pos0 (send *ri* :state :worldcoords "map"))
    (ros::ros-info "send ~A ~A ~A ~A" func x y d)
    (setq tm0 (ros::time-now))
    (send *ri* func x y d)
    ;; wait for 2 sec
    (send (instance ros::duration :init 2) :sleep)
    (setq tm1 (ros::time-now))
    (setq pos1 (send *ri* :state :worldcoords "map"))
    (setq diff-pos (send pos0 :difference-position pos1))
    (setq diff-rot (send pos0 :difference-rotation pos1))
    ;; if func is -no-wait, make sure that :go-pos-* returns immediately but move forward
    (when (substringp "NO-WAIT" (string func))
      (ros::ros-info "~A returns immediately ~A" func (send (ros::time- tm1 tm0) :to-sec))
      (ros::ros-info " difference-position: ~A -> ~A" diff-pos (norm diff-pos))
      (assert (< (abs (- (send (ros::time- tm1 tm0) :to-sec) 2.0)) 0.5))
      (assert (> (norm diff-pos) 25))
      (ros::ros-info "wait for result..")
      ;; wait for result
      (cond ((substringp "GO-POS-UNSAFE" (string func)) ;; FIXME
             (send *ri* :go-pos-unsafe-wait))
            (t
             (send *ri* :move-to-wait :retry 1 :frame-id (*ri* . base-frame-id))))
      (unix:sleep 1)
      (setq pos1 (send *ri* :state :worldcoords "map"))
      (setq diff-pos (send pos0 :difference-position pos1))
      (setq diff-rot (send pos0 :difference-rotation pos1))
      )
    (ros::ros-info "reached to the goal")
    (ros::ros-info " difference-position: ~A -> ~A ~A" diff-pos (- (elt diff-pos 0) (* 1000 x)) (- (elt diff-pos 1) (* 1000 y)))
    (ros::ros-info " difference-rotation: ~A -> ~A" diff-rot (shortest-angle (deg2rad d) (elt diff-rot 2)))
    ;; diff in 10cm
    (assert (< (abs (- (elt diff-pos 0) (* 1000 x))) 100)
            (format nil "go-pos moves incorrectly in x axis: ~A != ~A"
                    (* 1000 x) (elt diff-pos 0)))
    (assert (< (abs (- (elt diff-pos 1) (* 1000 y))) 100)
            (format nil "go-pos moves incorrectly in y axis: ~A != ~A"
                    (* 1000 y) (elt diff-pos 1)))
    (assert (< (abs (shortest-angle (deg2rad d) (elt diff-rot 2))) 0.8) ;; go-pos-unsafe-no-wait need 0.55... other cases fit within 0.1
            (format nil "go-pos moves incorrectly in d axis: ~A != ~A"
                    (deg2rad d) (elt diff-rot 2)))
    ))

(deftest test-go-pos
  (go-pos-func :func :go-pos))

(deftest test-go-pos-no-wait
  (go-pos-func :func :go-pos-no-wait))

(deftest test-go-pos-unsafe
  (go-pos-func :func :go-pos-unsafe))

(deftest test-go-pos-unsafe-no-wait
  (if (*ri* . move-base-trajectory-action)
      (go-pos-func :func :go-pos-unsafe-no-wait)
    (ros::ros-warn "SKIP TEST :go-pos-unsafe-no-wait requires move-base-trajectory-action")))

(run-all-tests)
(exit)

