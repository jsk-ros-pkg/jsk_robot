#!/usr/bin/env roseus

(load "package://roseus_smach/src/state-machine-ros.l")

(ros::roseus-add-msgs "jsk_recognition_msgs")

(setq find-person-msg (instance jsk_recognition_msgs::PeoplePoseArray :init))
(defun find-person-cb (msg)
  (setq find-person-msg msg))

(defun start-func (args)
  (let ()
    (set-alist 'description "Start Walking" args)
    :started))

(defun walk-func (args)
  (let ((last-time-seen-person 9999))
    (while (> last-time-seen-person 5)
      (setq last-time-seen-person (send (ros::time- (ros::time-now) (send find-person-msg :header :stamp)) :to-sec))
      (ros::ros-info "walk  : detect person ~A sec ago" last-time-seen-person)
      (ros::spin-once)
      (ros::sleep))
    ;; set description and goes to next state
    (set-alist 'description "Find Person" args)
    :find-person))

(defun find-person-func (args)
  (let ((last-time-seen-person 0))
    (while (< last-time-seen-person 5)
      (setq last-time-seen-person (send (ros::time- (ros::time-now) (send find-person-msg :header :stamp)) :to-sec))
      (ros::ros-info "person: detect person ~A sec ago" last-time-seen-person)
      (ros::spin-once)
      (ros::sleep))
    ; set description and goes to next state
    (set-alist 'description "Start Walking" args)
    :walk))

(defun lead-teleop-sm ()
  (let (sm)
    (setq sm
          (make-state-machine
           '((:start :started :walk)
             (:walk :find-person :find-person)
             (:walk :finished :end)
             (:find-person :walk :walk))
           '((:start 'start-func)
             (:end '(lambda (&rest args) :finished))
             (:walk 'walk-func) ;; function maps
             (:find-person 'find-person-func))
           '(:start)      ;; initial
           '(:end)       ;; goal
           ))
    (send sm :arg-keys 'description)
    sm))

(ros::roseus "lead-teleop-smach")
(ros::rate 1)
(ros::subscribe "/people_pose" jsk_recognition_msgs::PeoplePoseArray #'find-person-cb)
;; state machine
(exec-state-machine (lead-teleop-sm))
(ros::exit)
