(require :gen3_robotiq_2f_85 "package://kinovaeus/gen3_robotiq_2f_85.l")
(require :gen3_robotiq_2f_140 "package://kinovaeus/gen3_robotiq_2f_140.l")
(require :gen3_lite_gen3_lite_2f "package://kinovaeus/gen3_lite_gen3_lite_2f.l")

(defun kinova (&key (type :gen3_robotiq_2f_140) &rest args &allow-other-keys)
  (setq *kinova* (case type
                       (:gen3_robotiq_2f_85
                        (instance* gen3_robotiq_2f_85-robot :init args))
                       (:gen3_robotiq_2f_140
                        (instance* gen3_robotiq_2f_140-robot :init args))
                       (:gen3_lite_gen3_lite_2f
                        (instance* gen3_lite_gen3_lite_2f-robot :init args))
                       (t
                        (warning-message 1 "unknown kinova robot type ~A~%" type)
                        )))
  *kinova*)

(defmethod gen3_robotiq_2f_85-robot
  (:arm (&rest args) (send* self :rarm args))      ;; enable to call send *kinova* :arm :angle-vector
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
(defmethod gen3_robotiq_2f_140-robot
  (:arm (&rest args) (send* self :rarm args))      ;; enable to call send *kinova* :arm :angle-vector
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
(defmethod gen3_lite_gen3_lite_2f-robot
  (:arm (&rest args) (send* self :rarm args))      ;; enable to call send *kinova* :arm :angle-vector
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

