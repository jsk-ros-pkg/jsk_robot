#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")

(load "package://fetcheus/test/test-fetch-navigation-utils.l")

(init-unit-test)

(defun go-velocity-func (&key wait)
  (let ((vel-x 1) (vel-y 0) (vel-d 1.57)  ;; [m/s] [m/s] [rad/s]
        pos0 pos1 pos2 pos3 tm0 tm1 diff-pos vel-diff-pos)
    (setq pos0 (send *ri* :state :worldcoords "map"))
    (setq tm0 (ros::time-now))
    (ros::ros-info "(before go-velocity) current location ~A" pos0)
    (ros::ros-info "send :go-velocity ~A ~A ~A :wait ~A"  vel-x vel-y vel-d wait)
    (send *ri* :go-velocity vel-x vel-y vel-d 3000 :wait wait) ;; go-velocity blocks for 3 sec if wait is t
    (unless wait
      (send (instance ros::duration :init 2) :sleep)
      (setq tm1 (ros::time-now))
      (setq pos1 (send *ri* :state :worldcoords "map"))
      (setq diff-pos (send pos0 :difference-position pos1))
      (setq diff-rot (send pos0 :difference-rotation pos1))
      (ros::ros-info ":go-velocity :wait ~A returns immediately ~A" wait (send (ros::time- tm1 tm0) :to-sec))
      (ros::ros-info " difference-position: ~A -> ~A" diff-pos (norm diff-pos))
      (assert (< (abs (- (send (ros::time- tm1 tm0) :to-sec) 2.0)) 0.5))
      (assert (> (norm diff-pos) 25))
      (send (*ri* . move-base-trajectory-action) :wait-for-result)
      )
    (setq tm1 (ros::time-now))
    (unix:sleep 1) ;; make sure robot actually stops
    (setq pos1 (send *ri* :state :worldcoords "map"))
    (ros::ros-info "(after 1 sec )      current location ~A" pos1)
    (setq diff-pos (norm (send pos0 :difference-position pos1)))
    (setq vel-diff-pos
          (* 1000 (norm (float-vector vel-x vel-y))
             (send (ros::time- tm1 tm0) :to-sec)))
    (ros::ros-info " difference-position from position ~A" diff-pos)
    (ros::ros-info " difference-position from velocity ~A (~A sec)" vel-diff-pos (send (ros::time- tm1 tm0) :to-sec))
    (assert (>  vel-diff-pos diff-pos)
            (format nil "go-velocity moves too much: ~A" diff-pos))
    ))

(deftest test-go-velocity
  (if (*ri* . move-base-trajectory-action)
      (go-velocity-func)
    (ros::ros-warn "SKIP TEST: default behavior of :go-velocity is :wait nil, and this requires move-base-trajectory-action")))

(deftest test-go-velocity-wait
  (go-velocity-func :wait t))

(run-all-tests)
(exit)

