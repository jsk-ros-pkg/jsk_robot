#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "package://mycoboteus/mycobot-interface.l")


(deftest mycobot-eus-moveit-test
  (let ((ri (instance mycobot-interface :init))
        (co (instance collision-object-publisher :init))
        b c goal-pose diff-av )
    (format t "check reset-pose")
    (send ri :angle-vector (send ri :robot :reset-pose) 5000)
    (send ri :wait-interpolation)
    (format t "create reset-pose")
    ;; create collision object in moveit planning scence
    (setq b (make-cube 100 100 100))
    (setq c (make-coords :pos #f(200 0 200)))
    (send co :add-object b :relative-pose c :frame-id "link1")
    ;; plan and execute joint trajectory to the goal pose with collision avoidance
    (setq goal-pose #f(13.5698 -75.0476 -71.0797 24.4082 -172.768 168.488))
    (send ri :angle-vector-motion-plan goal-pose  :move-arm :rarm :total-time 5000)
    (send ri :wait-interpolation)
    (send co :clear-all) ;; remove all object
    (setq diff-av (send ri :sub-angle-vector (send ri :state :potentio-vector) goal-pose))
    (format t ":diff ~A, ~A~%"  (norm diff-av) (eps= (norm diff-av) 0.0 10.0))
    (assert (eps= (norm diff-av) 0.0 10.0) (format nil ":goal-pose, diff-av ~A" diff-av))
    ))

(run-all-tests)
(exit)
