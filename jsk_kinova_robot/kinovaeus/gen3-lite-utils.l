(require :gen3 "package://kinovaeus/gen3-lite.l")

(defun gen3-lite () (setq *gen3-lite* (instance gen3_lite_gen3_lite_2f-robot :init)))

(defmethod gen3_lite_gen3_lite_2f-robot
  (:arm (&rest args) (send* self :rarm args)) ;; enable to call send *gen3-lite* :arm :angle-vector
  (:go-grasp (&key (pos 0))
    (send self :right_finger_bottom_joint :joint-angle
          (rad2deg pos))
    (send self :right_finger_tip_joint :joint-angle
          (rad2deg (+ (* -0.676 pos) 0.149)))
    (send self :left_finger_bottom_joint :joint-angle
          (rad2deg (* -1 pos)))
    (send self :left_finger_tip_joint :joint-angle
          (rad2deg (+ (* -0.676 pos) 0.149))))
  (:start-grasp ()
    (send self :go-grasp :pos -0.05))
  (:stop-grasp ()
    (send self :go-grasp :pos 0.95))
  )
