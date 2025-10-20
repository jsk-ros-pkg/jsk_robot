(require :fr3 "package://panda_eus/models/fr3.l")

(defmethod fr3-robot
  (:arm (&rest args) (send* self :rarm args))  ;; Enable to call (send *fr3* :arm :angle-vector)
  (:start-grasp
    (arm &rest args &key (width 0.0) &allow-other-keys)
    (send* self :move-gripper arm width args))
  (:stop-grasp
    (arm &rest args &key (width 0.08) &allow-other-keys)
    (send* self :move-gripper arm width args))
  (:move-gripper
    (arm width &rest args)
    "Move the gripper to the target `width`.
Arguments:
- arm : :arm, :rarm, or :arms (only for compatibility with fr3-robot-interface)
- width : target distance between the fingers [m]
"
    (send-all
      (remove nil (mapcar
                    #'(lambda (jt)
                        (if (= (send jt :min-angle) (send jt :max-angle)) nil jt))
                    (send self :rarm :gripper :joint-list)))
      ;; Get joint list of gripper excluding fixed joints
      :joint-angle (* (/ width 2.0) 1000))))

(provide :fr3-utils)
