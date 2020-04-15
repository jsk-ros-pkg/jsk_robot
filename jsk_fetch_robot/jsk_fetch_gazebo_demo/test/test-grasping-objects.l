#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; force set *co* to aovid loading (instance collision-object-publisher :init))) that wait for '/apply_planning_scene' serice
(defclass dummy-collision-object-publisher :super propertied-object)
(defmethod dummy-collision-object-publisher
  (:init () self)
  (:wipe-all () t)
  )
(setq *co* (instance dummy-collision-object-publisher :init))
;; fake action goal/result id, skip wait-for-server
(in-package "ROS")
(defmethod ros::simple-action-client
  (:wait-for-server (&optional (timeout nil)) t)
(defmethod actionlib-comm-state
  (:update-result
   (msg)
   (setq latest-goal-status (send msg :status))
   (setq latest-result msg)
   (ros::ros-info "[~A] update-result goal id: ~A ~A~%" (send self :name) (send msg :status :goal_id :id) state)
   (if action-goal (send msg :status :goal_id :id (send action-goal :goal_id :id)))
   (ros::ros-info "[~A]      replaced goal id: ~A ~A~%" (send self :name) (send msg :status :goal_id :id) state)
   (send self :update-status
         (instance actionlib_msgs::GoalStatusArray :init :status_list (list (send msg :status))))
   (setq state 'ros::*comm-state-done*))
   (ros::ros-info "[~A] update-result state: ~A ~A" (send self :name) (send msg :status :goal_id :id) state)
   ))
(in-package "USER")
;;
(load "package://jsk_fetch_gazebo_demo/euslisp/demo.l")

(deftest grasp-client ()
  (let (ret cube grasps)
    (setq grasping-client (instance grasping-client :init
                                    :find-topic "grasp_objects_test/basic_grasping_perception/find_objects"))
    (ros::ros-warn "Picking object...")
    (send grasping-client :update-scene)
    (setq ret (send grasping-client :get-graspable-cube))
    (ros::ros-warn "Graspable object results: ~A" ret)
    (setq cube (car ret))
    (setq grasps (cadr ret))
    (ros::ros-warn "Graspable cube ~A / grasps" cube grasps)
    ))

(run-all-tests)
(exit)

