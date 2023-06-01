(require :dual_panda "package://panda_eus/models/dual_panda.l")

(defmethod dual_panda-robot
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
- arm : :rarm, :larm, or :arms
- width : target distance between the fingers [m]
"
    (let ((arms (case arm
                  ((:rarm :larm) (list arm))
                  (:arms (list :rarm :larm)))))
      (dolist (am arms)
        (send-all
          (remove nil (mapcar
                        #'(lambda (jt)
                            (if (= (send jt :min-angle) (send jt :max-angle)) nil jt))
                        (send self am :gripper :joint-list)))
          ;; Get joint list of gripper excluding fixed joints
          :joint-angle (* (/ width 2.0) 1000))))))

(provide :dual_panda-utils)
