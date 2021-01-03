(require :gen3 "package://kinovaeus/gen3.l")

(defun gen3 () (setq *gen3* (instance gen3_robotiq_2f_85-robot :init)))

(defmethod gen3_robotiq_2f_85-robot
  (:arm (&rest args) (send* self :rarm args)) ;; enable to call send *gen3* :arm :angle-vector
  (:go-grasp (&key (pos 0))
    (let ((pos-deg (rad2deg pos)))
      (send self :finger_joint :joint-angle pos-deg)
      (send self :left_inner_finger_joint :joint-angle (* -1 pos-deg))
      (send self :left_inner_knuckle_joint :joint-angle pos-deg)
      (send self :right_inner_finger_joint :joint-angle (* -1 pos-deg))
      (send self :right_inner_knuckle_joint :joint-angle pos-deg)
      (send self :right_outer_knuckle_joint :joint-angle pos-deg)))
  (:start-grasp ()
    (send self :go-grasp :pos 0.8))
  (:stop-grasp ()
    (send self :go-grasp :pos 0.0))
  )
