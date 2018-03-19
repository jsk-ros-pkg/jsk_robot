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
    (send robot :untuck-pose)
    (send robot :larm :move-end-pos #f(0 -120 0) :world)
    (send robot :rarm :move-end-pos #f(0 120 0) :world)
    (assert (send robot :self-collision-check))
    ))

(setq *pod-bin* '(#f(950.0 280.0 1695.0) #f(950.0 -2.449294e-14 1695.0) #f(950.0 -280.0 1695.0) #f(950.0 280.0 1465.0) #f(950.0 -2.449294e-14 1465.0) #f(950.0 -280.0 1465.0) #f(950.0 280.0 1235.0) #f(950.0 -2.449294e-14 1235.0) #f(950.0 -280.0 1235.0) #f(950.0 280.0 975.0) #f(950.0 -2.449294e-14 975.0) #f(950.0 -280.0 975.0)))

(setq *pod-bin-cube* (list
                       (send (make-cube 370 230 200) :locate #f(1210 270  1760) :world) ;; bin a
                       (send (make-cube 370 270 200) :locate #f(1210 0    1760) :world) ;; bin b
                       (send (make-cube 370 230 200) :locate #f(1210 -270 1760) :world) ;; bin c
                       (send (make-cube 370 230 200) :locate #f(1210 270  1510) :world) ;; bin d
                       (send (make-cube 370 270 200) :locate #f(1210 0    1510) :world) ;; bin e
                       (send (make-cube 370 230 200) :locate #f(1210 -270 1510) :world) ;; bin f
                       (send (make-cube 370 230 200) :locate #f(1210 270  1280) :world) ;; bin g
                       (send (make-cube 370 270 200) :locate #f(1210 0    1280) :world) ;; bin h
                       (send (make-cube 370 230 200) :locate #f(1210 -270 1280) :world) ;; bin i
                       (send (make-cube 370 230 200) :locate #f(1210 270  1020) :world) ;; bin j
                       (send (make-cube 370 270 200) :locate #f(1210 0    1020) :world) ;; bin k
                       (send (make-cube 370 230 200) :locate #f(1210 -270 1020) :world) ;; bin l
                       ))

(setq *pod-bin-ik-limit*
      (list 1300 1200 1300 0 0 1200 0 0 1200 0 1300 1100))

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
                      :debug-view :no-message) "fail to solve ik")
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
                      :debug-view :no-message) "fail to solve ik")
        )) ;; do list
    )) ;; let

;; test to check the robot back to the original pose, if the ik failed
(deftest test-robot-ik-revert
  (let (robot av)
    (setq robot (instance baxter-robot :init))
    (send robot :reset-pose)
    (send robot :larm :angle-vector #f(0 0 0 0 0 0 0))
    (send robot :rarm :wrist-p :joint-angle 10 :relative t)
    (setq av (send robot :angle-vector))
    (send robot :rarm :inverse-kinematics (make-coords)) ;; failing is ok
    (assert (eps-v= (send robot :angle-vector) av 5) "unable to revert after ik failed")
    (send robot :larm :inverse-kinematics (make-coords)) ;; failing is ok
    (assert (eps-v= (send robot :angle-vector) av 5) "unable to revert after ik failed")
    ))

(deftest test-robot-ik-bin-cube
  (let (robot)
    (setq robot (instance baxter-robot :init))
    (send (send robot :rarm :end-coords)
          :newcoords (make-coords :pos #f(50 0 310)
                                  :rpy (float-vector 0 -pi/2 0)))
    (send (send robot :larm :end-coords)
          :newcoords (make-coords :pos #f(50 0 310)
                                  :rpy (float-vector 0 -pi/2 0)))
    (send robot :locate #f(0 0 950) :world)
    (objects (append (list robot) *pod-bin-cube*))
    (setq step-dist 200)
    (dolist (cube-data (mapcar #'list *pod-bin-ik-limit* *pod-bin-cube*))
      (setq limit (car cube-data))
      (setq cube (cadr cube-data))
      (setq x (elt (send (send cube :worldcoords) :pos) 0))
      (setq y (elt (send (send cube :worldcoords) :pos) 1))
      (setq z (elt (send (send cube :worldcoords) :pos) 2))
      (setq dx (x-of-cube cube))
      (setq dy (y-of-cube cube))
      (setq dz (z-of-cube cube))
      (do ((tmp-x (+ (- x (/ dx 2)) (/ step-dist 2)) (+ tmp-x step-dist)))
        ((> tmp-x (+ x (/ dx 2))) t)
        (do ((tmp-y (+ (- y (/ dy 2)) (/ step-dist 2)) (+ tmp-y step-dist)))
          ((> tmp-y (+ y (/ dy 2))) t)
          (do ((tmp-z (+ (- z (/ dz 2)) (/ step-dist 2)) (+ tmp-z step-dist)))
            ((> tmp-z (+ z (/ dz 2))) t)
            (setq pos (float-vector tmp-x tmp-y tmp-z))
            (assert
              (or
                (send robot :rarm :inverse-kinematics
                  (make-cascoords :pos pos)
                  :rotation-axis t
                  :debug-view :no-message)
                (send robot :larm :inverse-kinematics
                  (make-cascoords :pos pos)
                  :rotation-axis t
                  :debug-view :no-message)
                (> tmp-x limit))
              "fail to solve ik")
            )
          )
        )
      )
    )
  )

(deftest test-robot-ik-safe-bin-cube
  (let (robot)
    (setq robot (instance baxter-robot-safe :init))
    (send (send robot :rarm :end-coords)
          :newcoords (make-coords :pos #f(50 0 310)
                                  :rpy (float-vector 0 -pi/2 0)))
    (send (send robot :larm :end-coords)
          :newcoords (make-coords :pos #f(50 0 310)
                                  :rpy (float-vector 0 -pi/2 0)))
    (send robot :locate #f(0 0 950) :world)
    (objects (append (list robot) *pod-bin-cube*))
    (setq step-dist 200)
    (dolist (cube-data (mapcar #'list *pod-bin-ik-limit* *pod-bin-cube*))
      (setq limit (car cube-data))
      (setq cube (cadr cube-data))
      (setq x (elt (send (send cube :worldcoords) :pos) 0))
      (setq y (elt (send (send cube :worldcoords) :pos) 1))
      (setq z (elt (send (send cube :worldcoords) :pos) 2))
      (setq dx (x-of-cube cube))
      (setq dy (y-of-cube cube))
      (setq dz (z-of-cube cube))
      (do ((tmp-x (+ (- x (/ dx 2)) (/ step-dist 2)) (+ tmp-x step-dist)))
        ((> tmp-x (+ x (/ dx 2))) t)
        (do ((tmp-y (+ (- y (/ dy 2)) (/ step-dist 2)) (+ tmp-y step-dist)))
          ((> tmp-y (+ y (/ dy 2))) t)
          (do ((tmp-z (+ (- z (/ dz 2)) (/ step-dist 2)) (+ tmp-z step-dist)))
            ((> tmp-z (+ z (/ dz 2))) t)
            (setq pos (float-vector tmp-x tmp-y tmp-z))
            (assert
              (or
                (and
                  (send robot :reset-pose)
                  (send robot :rarm :inverse-kinematics
                    (make-cascoords :pos pos)
                    :rotation-axis t
                    :debug-view :no-message)
                  )
                (and
                  (send robot :reset-pose)
                  (send robot :larm :inverse-kinematics
                    (make-cascoords :pos pos)
                    :rotation-axis t
                    :debug-view :no-message)
                  )
                (> tmp-x limit)
                )
                "fail to solve ik"
              )
            )
          )
        )
      )
    )
  )

(deftest test-robot-l/r-reverse
  (let (robot reversed)
    (setq robot (instance baxter-robot :init))
    (setq reversed (send robot :l/r-reverse (send robot :larm :angle-vector)))
    (dotimes (i (length (send robot :rarm :angle-vector)))
      (assert (= (elt (send robot :rarm :angle-vector) i) (elt reversed i))))
    ))

(when (string> (unix::getenv "ROS_DISTRO") "hydro")
(load "package://baxtereus/baxter-interface.l")
(deftest test-baxter-interface
  (let (robot ri msg)
    (setq ri (instance baxter-interface :init))
    (send ri :state :potentio-vector)
    (setq robot (instance baxter-robot :init))
    (send ri :angle-vector (send robot :angle-vector))
    (send ri :angle-vector-sequence (list (send robot :reset-pose) (send robot :reset-manip-pose)))
    (setq msg (instance sensor_msgs::JointState :init))
    (send msg :name '("torso_t0" "head_pan" "left_s0" "left_s1" "left_e0" "left_e1" "left_w0" "left_w1" "left_w2" "right_s0" "right_s1" "right_e0" "right_e1" "right_w0" "right_w1" "right_w2"))
    (send msg :position (instantiate float-vector (length (send msg :name))))
    (send msg :velocity (instantiate float-vector (length (send msg :name))))
    (send msg :effort (instantiate float-vector (length (send msg :name))))
    (send ri :ros-state-callback msg)
    ;; torso_t0 should not be contained in robot-state
    (assert (not (member "torso_t0" (cdr (assoc :name (assoc 'robot-state (send ri :slots)))) :test #'string=)))
    ))

(deftest test-baxter-init
  (baxter-init))
) ;; when


(run-all-tests)
(exit)
