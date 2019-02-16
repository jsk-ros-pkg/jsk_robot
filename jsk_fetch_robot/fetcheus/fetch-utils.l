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
  (:start-grasp
    ()
    (send self :go-grasp :pos 0))
  (:stop-grasp
    ()
    (send self :go-grasp :pos 0.1))
  )
