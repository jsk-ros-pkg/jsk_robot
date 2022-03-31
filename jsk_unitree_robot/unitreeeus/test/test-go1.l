#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(require "package://unitreeeus/unitree-interface.l")

(init-unit-test)
(setq lisp::*exit-on-fatal-error* nil) ;; uncommnent when debugging

(deftest test-body-pose
  (let (robot target)
    (setq *ri* (instance go1-interface :init))
    (setq robot (go1))
    (objects (list robot))
    (dotimes (j 3)
      (dotimes (i 50)
        (setq target (float-vector 0 0 0))
        (setf (elt target j) (if (< i 25) 0.02 -0.02))
        (send robot :body-pose target)
        (send *ri* :body-pose (send robot :worldcoords))
        (send *irtviewer* :draw-objects)
        ))
    ))

(deftest test-look-at
  (let (robot target)
    (setq *ri* (instance go1-interface :init))
    (setq robot (go1))
    (setq target (make-cube 50 50 50 :pos #f(800 0 0)))
    (objects (list *go1* target))
    (dotimes (i 100)
      (send target :locate (float-vector 800 (* 200 (sin (* pi (/ i 20.0)))) (* 200 (sin (* pi (/ i 40.0))))) :world)
      (send robot :look-at (send target :worldpos) :revert-if-fail nil)
      (send *ri* :body-pose (send robot :worldcoords))
      (send *irtviewer* :draw-objects)
      )
    ))


(run-all-tests)
;; (exit) ;; comment when debugging
