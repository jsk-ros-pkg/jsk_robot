#!/usr/bin/env roseus
;; vim: set ft=lisp:

(require :unittest "lib/llib/unittest.l")
(require "package://baxtereus/baxter.l")


;; init
;; ----------------------------------------------------------------------------
(init-unit-test)
;; test data
(setq *end-coords* (make-hash-table))
(sethash :larm *end-coords* (make-coords :pos #f(708.974 522.351 180.297)
                                         :rpy #f(-1.129 0.56 -1.936)))
(sethash :rarm *end-coords* (make-coords :pos #f(708.974 -522.351 180.297)
                                         :rpy #f(1.129 0.56 1.936)))


;; define tests
;; ----------------------------------------------------------------------------
(deftest
  test-end-coords-position
  (setq *baxter* (instance baxter-robot :init))
  (setq *robot* *baxter*)

  (dolist (arm '(:larm :rarm))
    (let (dist)
      (setq dist
            (distance
              (send (gethash arm *end-coords*) :worldpos)
              (send *robot* arm :end-coords :worldpos)
              )
            )
      (format t "~a end-coords position distance: ~a~%" arm dist)
      (assert (< dist 0.01))  ;; distance should be less than 0.01
      )
    )
  )


(deftest
  test-end-coords-rotation
  (setq *baxter* (instance baxter-robot :init))
  (setq *robot* *baxter*)

  (dolist (arm '(:larm :rarm))
    (let (rpy-dist)
      (dolist (rpy (rpy-angle (send *robot* arm :end-coords :worldrot)))
        (setq rpy-dist
              (mapcar
                #'(lambda (x) (distance (apply 'float-vector x) (apply 'float-vector rpy)))
                (rpy-angle (send (gethash arm *end-coords*) :worldrot))
                )
              )
        (format t "~a end-coords rpy distance: ~a~%" arm rpy-dist)
        ;; distance should be less than 0.01
        (assert (some #'identity (mapcar #'(lambda (x) (< x 0.01)) rpy-dist)))
        )
      )
    )
  )


;; main
;; ----------------------------------------------------------------------------
(run-all-tests)
(exit)
