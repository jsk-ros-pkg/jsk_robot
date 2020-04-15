#!/usr/bin/env roseus

;; This script is roseus version of demo.py in fetch_gazebo_demo
;; https://github.com/fetchrobotics/fetch_gazebo/blob/gazebo9/fetch_gazebo_demo/scripts/demo.py

(require "package://fetcheus/fetch-interface.l")
;; hotfix for https://github.com/jsk-ros-pkg/jsk_robot/pull/1213#issuecomment-612976152, see https://github.com/jsk-ros-pkg/jsk_roseus/issues/574
(make-package "GRASPING_MSGS")
(load "package://roseus/ros/grasping_msgs/msg/Object.l")
;;
(ros::load-ros-manifest "grasping_msgs")

;; Remove (shape-msg->xxx) functions after https://github.com/jsk-ros-pkg/jsk_roseus/pull/640 is released
(defun shape-msg->shape (msg)
  "Convert shape_msgs::SolidPrimitive to euslisp object"
  (let ((type (send msg :type)))
    (cond
     ((eq type shape_msgs::SolidPrimitive::*BOX*)
      (shape-msg->cube msg))
     ((eq type shape_msgs::SolidPrimitive::*SPHERE*)
      (shape-msg->sphere msg))
     ((eq type shape_msgs::SolidPrimitive::*CYLINDER*)
      (shape-msg->cylinder msg))
     (t
      (error "unknown type ~A" type)))))
(defun shape-msg->cube (msg)
  "Convert shape_msgs::SolidPrimitive to euslisp cube object"
  (let* ((scale (send msg :dimensions))
         (cb (make-cube (* (elt scale 0) 1000)
                        (* (elt scale 1) 1000)
                        (* (elt scale 2) 1000))))
    cb))
(defun shape-msg->sphere (msg)
  "Convert shape_msgs::SolidPrimitive to euslisp sphere object"
  (let* ((scale (send msg :dimensions))
         (cp (make-sphere (* (elt scale 0) 1000))))
    cp))
(defun shape-msg->cylinder (msg)
  "Convert shape_msgs::SolidPrimitive to euslisp cylinder object"
  (let* ((scale (send msg :dimensions))
         (height (* (elt scale 0) 1000))
         (radius (* (elt scale 1) 1000))
         (cyl (make-cylinder radius height)))
    cyl))

(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))
(unless (boundp '*co*)
  (setq *co* (instance collision-object-publisher :init)))

(defclass grasping-client
  :super propertied-object
  :slots (find-client objects support-surfaces)
  )

(defmethod grasping-client
  (:init (&key (find-topic "basic_grasping_perception/find_objects"))
    (let ()
      (ros::ros-info (format nil "Waiting for ~A ..." find-topic))
      (setq find-client (instance ros::simple-action-client :init
                                  find-topic
                                  grasping_msgs::FindGraspableObjectsAction))
      (send find-client :wait-for-server)))
  (:update-scene ()
    (let ((goal (instance grasping_msgs::FindGraspableObjectsGoal :init))
          (map-to-frame (send *tfl* :lookup-transform "map" "base_link" (ros::time 0)))
          find-result primitive-pose scene-object height)
      ;; Find objects
      (send goal :plan_grasps t)
      (send find-client :send-goal goal)
      (send find-client :wait-for-result :timeout 5.0)
      (setq find-result (send find-client :get-result))
      ;; Remove previous objects
      (send *co* :wipe-all)
      ;; Insert objects to scene
      (dolist (obj (send find-result :objects))
        (setq primitive-pose
              (ros::tf-pose->coords (car (send (send obj :object) :primitive_poses))))
        (setq scene-object
              (shape-msg->shape (car (send (send obj :object) :primitives))))
        (send scene-object :transform primitive-pose :world)
        (send scene-object :transform map-to-frame :world)
        (send *co* :add-object scene-object :frame-id "map"))
      (dolist (obj (send find-result :support_surfaces))
        (setq primitive-pose
              (ros::tf-pose->coords (car (send obj :primitive_poses))))
        (setq height (elt (send primitive-pose :worldpos) 2))
        (setq scene-object
              (shape-msg->cube (car (send obj :primitives))))
        (setq scene-object
              (make-cube (x-of-cube scene-object)
                         (* 1.5 1000)
                         (+ (z-of-cube scene-object) height)))
        (send scene-object :transform primitive-pose :world)
        (send scene-object :transform map-to-frame :world)
        (send scene-object :translate (float-vector 0 0 (/ height -2.0)) :world)
        (send *co* :add-object scene-object :frame-id "map"))
      ;; Store for grasping
      (setq objects (send find-result :objects))
      (setq support-surfaces (send find-result :support_surfaces))))
  (:get-graspable-cube ()
    (let (graspable)
      (dolist (obj objects)
        (setq continue-flag nil)
        ;; need grasps
        (if (or (< (length (send obj :grasps)) 1) continue-flag)
          (setq continue-flag t))
        ;; check size
        (if (or (< (elt (send (car (send (send obj :object) :primitives)) :dimensions) 0) 0.05)
                (> (elt (send (car (send (send obj :object) :primitives)) :dimensions) 0) 0.07)
                (< (elt (send (car (send (send obj :object) :primitives)) :dimensions) 0) 0.05)
                (> (elt (send (car (send (send obj :object) :primitives)) :dimensions) 0) 0.07)
                (< (elt (send (car (send (send obj :object) :primitives)) :dimensions) 0) 0.05)
                (> (elt (send (car (send (send obj :object) :primitives)) :dimensions) 0) 0.07)
                continue-flag)
          (setq continue-flag t))
        ;; has to be on table
        (if (or (< (send (car (send (send obj :object) :primitive_poses)) :position :z) 0.5)
                continue-flag)
          (setq continue-flag t))
        (unless continue-flag
          (return-from :get-graspable-cube (list (send obj :object) (send obj :grasps)))))
      ;; nothing detected
      (return-from :get-graspable-cube (list nil nil))))
  )

(defun main ()
  ;; Create *ri* to use fetch with MoveIt! and Navigation stack
  (fetch-init)

  ;; Move the base to be in front of the table
  ;; Demonstrates the use of the navigation stack
  (ros::ros-info "Moving to table...")
  (send *ri* :move-to (make-coords :pos #f(2250 3118 0)) :frame-id "map")
  (send *ri* :move-to (make-coords :pos #f(2750 3118 0)) :frame-id "map")

  ;; Raise the torso using just a controller
  (ros::ros-info "Raising torso...")
  (send *fetch* :reset-pose)
  (send *fetch* :torso :waist-z :joint-angle 400)
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)

  ;; Point the head at the cube we want to pick
  (setq frame-to-map (send *tfl* :lookup-transform "base_link" "map" (ros::time 0)))
  (send *fetch* :look-at-target
        (send (make-coords :pos #f(3700 3180 0)) :transform frame-to-map :world))
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)

  ;; Get block to pick
  (setq grasping-client (instance grasping-client :init))
  (while t
    (ros::ros-info "Picking object...")
    (send grasping-client :update-scene)
    (setq ret (send grasping-client :get-graspable-cube))
    (setq cube (car ret))
    (setq grasps (cadr ret))
    (if cube
      (progn
        ;; Pick the block
        ;; NOTE We do not use pick function like demo.py in fetch_gazebo_demo
        (setq primitive-pose
              (ros::tf-pose->coords (car (send cube :primitive_poses))))
        (setq scene-object
              (shape-msg->shape (car (send cube :primitives))))
        (send scene-object :transform primitive-pose :world)
        (send scene-object :rotate pi/2 :y :local)
        (send *fetch* :inverse-kinematics (send scene-object :copy-worldcoords) :use-torso nil)
        (send *fetch* :rarm :move-end-pos #f(0 0 150) :world :use-torso nil)
        (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
        (send *ri* :wait-interpolation)
        (send *ri* :stop-grasp :wait t)
        (send *fetch* :rarm :move-end-pos #f(0 0 -120) :world :use-torso nil)
        (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
        (send *ri* :wait-interpolation)
        (send *ri* :start-grasp :wait t)
        (return))
      ;; TODO end
      (ros::ros-warn "Perception failed.")))

  ;; Intermediate pose for MoveIt planning and avoid collision between table and cube
  (send *fetch* :rarm :move-end-pos #f(0 500 100) :world :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Tuck the arm
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :arm-controller)
  (send *ri* :wait-interpolation)

  ;; Lower torso
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000 :torso-controller)
  (send *ri* :wait-interpolation)
  (send *co* :wipe-all)

  ;; Move to second table
  (ros::ros-info "Moving to second table...")
  (send *ri* :move-to (make-coords :pos #f(-3530 3750 0) :rpy (float-vector pi/2 0 0)) :frame-id "map")
  (send *ri* :move-to (make-coords :pos #f(-3530 4150 0) :rpy (float-vector pi/2 0 0)) :frame-id "map")

  ;; Raise the torso using just a controller
  (ros::ros-info "Raising torso...")
  (send *fetch* :torso :waist-z :joint-angle 400)
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)

  ;; Place the block
  ;; NOTE We do not use place function like demo.py in fetch_gazebo_demo
  (setq frame-to-map (send *tfl* :lookup-transform "base_link" "map" (ros::time 0)))
  (send *fetch* :look-at-target
        (send (make-coords :pos #f(-3000 5000 0)) :transform frame-to-map :world))
  (send *ri* :angle-vector (send *fetch* :angle-vector))
  (send *ri* :wait-interpolation)
  (while t
    (ros::ros-info "Placing object...")
    (send grasping-client :update-scene)
    (send *fetch* :inverse-kinematics (send scene-object :copy-worldcoords) :use-torso nil)
    (send *fetch* :rarm :move-end-pos #f(0 0 150) :world :use-torso nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 10000) ;; move arm slowly not to drop cube
    (send *ri* :wait-interpolation)
    (send *fetch* :rarm :move-end-pos #f(0 0 -120) :world :use-torso nil)
    (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-grasp :wait t)
    (return))
  ;; NOTE end

  ;; Intermediate pose for MoveIt planning and avoid collision between table and cube
  (send *fetch* :rarm :move-end-pos #f(0 500 100) :world :use-torso nil)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  ;; Tuck the arm, lower the torso
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000 :arm-controller)
  (send *ri* :wait-interpolation)
  (send *co* :wipe-all)

  (exit)
  )
