#!/usr/bin/env roseus
;;;
;;; test code for sciurus17
;;;

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "package://sciurus17eus/sciurus17-utils.l")

(deftest test-sciurus17-robot-model
  (setq *sciurus17* (instance sciurus17-robot :init))
  (assert *sciurus17* "creating sciurus17 model")
  (assert (derivedp *sciurus17* cascaded-link)
          "sciurus17 model is cascaded-link")
  )

(deftest test-sciurus17-links
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (let ((links (send *sciurus17* :links)))
    (assert links "sciurus17 has links")
    (assert (> (length links) 0) "sciurus17 has at least one link")
    (format t "sciurus17 has ~A links~%" (length links))
    ))

(deftest test-sciurus17-joints
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (let ((joints (send *sciurus17* :joint-list)))
    (assert joints "sciurus17 has joints")
    (assert (> (length joints) 0) "sciurus17 has at least one joint")
    (format t "sciurus17 has ~A joints~%" (length joints))
    ))

(deftest test-sciurus17-arms
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (assert (send *sciurus17* :larm) "sciurus17 has left arm")
  (assert (send *sciurus17* :rarm) "sciurus17 has right arm")
  (assert (> (length (send *sciurus17* :larm :joint-list)) 0)
          "left arm has joints")
  (assert (> (length (send *sciurus17* :rarm :joint-list)) 0)
          "right arm has joints")
  (format t "left arm has ~A joints~%" (length (send *sciurus17* :larm :joint-list)))
  (format t "right arm has ~A joints~%" (length (send *sciurus17* :rarm :joint-list)))
  )

(deftest test-sciurus17-head
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (assert (send *sciurus17* :head) "sciurus17 has head")
  (assert (> (length (send *sciurus17* :head :joint-list)) 0)
          "head has joints")
  (format t "head has ~A joints~%" (length (send *sciurus17* :head :joint-list)))
  )

(deftest test-sciurus17-torso
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (assert (send *sciurus17* :torso) "sciurus17 has torso")
  (assert (> (length (send *sciurus17* :torso :joint-list)) 0)
          "torso has joints")
  (format t "torso has ~A joints~%" (length (send *sciurus17* :torso :joint-list)))
  )

(deftest test-sciurus17-reset-pose
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (send *sciurus17* :reset-pose)
  (assert t "reset-pose works")
  )

(deftest test-sciurus17-angle-vector
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (let ((av (send *sciurus17* :angle-vector)))
    (assert av "angle-vector returns value")
    (assert (vectorp av) "angle-vector returns vector")
    (assert (= (length av) (length (send *sciurus17* :joint-list)))
            "angle-vector length matches joint count")
    ))

(deftest test-sciurus17-inverse-kinematics-rarm
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (send *sciurus17* :reset-pose)
  (let* ((target-coords (send (send *sciurus17* :rarm :end-coords) :copy-worldcoords))
         (result (send *sciurus17* :rarm :inverse-kinematics target-coords)))
    (assert result "rarm inverse-kinematics succeeds")
    ))

(deftest test-sciurus17-inverse-kinematics-larm
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (send *sciurus17* :reset-pose)
  (let* ((target-coords (send (send *sciurus17* :larm :end-coords) :copy-worldcoords))
         (result (send *sciurus17* :larm :inverse-kinematics target-coords)))
    (assert result "larm inverse-kinematics succeeds")
    ))

(deftest test-sciurus17-collision-check
  (unless (boundp '*sciurus17*)
    (setq *sciurus17* (instance sciurus17-robot :init)))
  (send *sciurus17* :reset-pose)
  ;; just check if collision-check method exists and can be called
  (assert (send *sciurus17* :self-collision-check) "self-collision-check method exists")
  )

(run-all-tests)
(exit)
