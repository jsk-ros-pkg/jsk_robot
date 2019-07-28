;;
;;
;;
(require :fetch "package://fetcheus/fetch.l")

(defmethod fetch-robot
  (:inverse-kinematics
   (target-coords &rest args &key link-list move-arm (use-torso t) move-target &allow-other-keys)
   (unless move-arm (setq move-arm :rarm))
   (unless move-target (setq move-target (send self :rarm :end-coords)))
   (unless link-list
     (setq link-list (send self :link-list (send move-target :parent)
                           (unless use-torso (car (send self :rarm))))))
   (send-super* :inverse-kinematics target-coords
                :move-target move-target
                :link-list link-list
                args))
  (:go-grasp
    (&key (pos 0)) ;; pos is between 0.0 and 0.1
      (send self :l_gripper_finger_joint :joint-angle (/ (* pos 1000) 2)) ;; m -> mm
      (send self :r_gripper_finger_joint :joint-angle (/ (* pos 1000) 2))
      (return-from :go-grasp t))
  (:gripper-angle
   ()
   (+ (/ (send self :l_gripper_finger_joint :joint-angle) 1000.0) ;; mm -> m
      (/ (send self :r_gripper_finger_joint :joint-angle) 1000.0)))
  ;; this function is mainly copied from https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/master/pr2eus/pr2-utils.l#L280
  ;; in the future, we should integrate these two codes into upstream code.
  (:start-grasp
   (&optional arg1 arg2)
   (let ((angle 0) target)
     (cond
      ((or (derivedp arg1 body) (derivedp arg1 cascaded-coords))
       (setq target arg1)
       (let ((gripper-angle (send self :gripper-angle)))
         (while t
           (send self :go-grasp :pos (- (send self :gripper-angle) 0.001))
           (when (= (send self :gripper-angle) 0)
             (send self :go-grasp :pos gripper-angle)
             (return)
             )
           (when  (or (< 0 (pqp-collision-check
                            (elt (send self :rarm :gripper :links) 0) target)) ;; l_gripper_finger_link
                      (< 0 (pqp-collision-check
                            (elt (send self :rarm :gripper :links) 1) target))) ;; r_gripper_finger_link
             (send self :go-grasp :pos (+ (send self :gripper-angle) 0.001))
             (ros::ros-info "catched:~A" target)
             (send self :grasping-obj target)
             (send (send self :r_gripper_finger_joint) :parent-link :assoc target)
             (return)
             ))))
      (t
       (when (numberp arg1)
         (setq angle arg1 target arg2))
       (send self :go-grasp :pos angle)
       (when target
         (send self :grasping-obj target)
         (send (send self :r_gripper_finger_joint) :parent-link :assoc target)
         )))))
  ;; this function is mainly copied from https://github.com/jsk-ros-pkg/jsk_pr2eus/blob/master/pr2eus/pr2-utils.l#L332
  ;; in the future, we should integrate these two codes into upstream code.
  (:stop-grasp
   (&optional arg1 arg2)
   (let (angle target finger-joint)
     (cond
      ((and (numberp arg1) (derivedp arg2 body))
       (setq angle arg1 target arg2)
       (when (eq (send self :grasping-obj) target)
         (send self :grasping-obj nil)
         (send (send self :r_gripper_finger_joint) :parent-link :dissoc target))
       (send self :go-grasp :pos angle))
      ((numberp arg1)
       (setq angle arg1)
       (send self :go-grasp :pos angle))
      (t
       (when (derivedp arg1 body) (setq target arg1))
       (when (null target)
         (setq target (send self :grasping-obj)))
       (send self :grasping-obj nil)
       (send (send self :r_gripper_finger_joint) :parent-link :dissoc target)
       (send self :go-grasp :pos 0.1)
         ))))
  (:grasping-obj
   (&optional (target t))
   (cond
    ((or (derivedp target body) (derivedp target cascaded-coords))
     (setf (get self 'rarm-grasping-obj) target))
    (target
     (get self 'rarm-grasping-obj))
    ((null target) ;; setq grasping-obj nil
     (setf (get self 'rarm-grasping-obj) nil))
    (t
     )))
  (:collision-check-pairs
   (&key ((:links ls) (cons (car links) (all-child-links (car links)))))
   (let (pairs l neighbors
               (exclude-pairs
                (list (cons base_link_lk bellows_link2_lk)
                      (cons base_link_lk bellows_link_lk)
                      (cons torso_fixed_link_lk torso_lift_link_lk)
                      (cons torso_fixed_link_lk bellows_link2_lk)
                      (cons torso_fixed_link_lk bellows_link_lk)
                      (cons bellows_link2_lk bellows_link_lk)
                      (cons r_gripper_finger_link_lk l_gripper_finger_link_lk))))
     (while (setq l (pop ls))
       (setq neighbors (remove nil
                               (append
                                (send l :descendants)
                                (send l :child-links)
                                (list (send l :parent-link) (send l :parent)))))
       (dolist (l2 ls)
         (if (and (not (memq l2 neighbors))
                  (not (member (cons l l2) exclude-pairs :test #'equal)))
           (push (cons l l2) pairs))))
     pairs))
  )
