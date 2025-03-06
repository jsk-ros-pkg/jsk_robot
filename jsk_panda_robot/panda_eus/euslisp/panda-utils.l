(require :panda "package://panda_eus/models/panda.l")

(defmethod panda-robot
  (:arm (&rest args) (send* self :rarm args))  ;; Enable to call (send *panda* :arm :angle-vector)
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
- arm : :arm, :rarm, or :arms (only for compatibility with panda-robot-interface)
- width : target distance between the fingers [m]
"
    (send-all
      (remove nil (mapcar
                    #'(lambda (jt)
                        (if (= (send jt :min-angle) (send jt :max-angle)) nil jt))
                    (send self :rarm :gripper :joint-list)))
      ;; Get joint list of gripper excluding fixed joints
      :joint-angle (* (/ width 2.0) 1000))))

(provide :panda-utils)
