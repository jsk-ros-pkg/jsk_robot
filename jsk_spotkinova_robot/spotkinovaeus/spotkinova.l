;; Mainly copied from kinovaeus/kinova.l

(require :spot_gen3_robotiq_2f_85 "package://spotkinovaeus/spot_gen3_robotiq_2f_85.l")
(require :spot_gen3_robotiq_2f_140 "package://spotkinovaeus/spot_gen3_robotiq_2f_140.l")
(require :spot_gen3_lite_gen3_lite_2f "package://spotkinovaeus/spot_gen3_lite_gen3_lite_2f.l")

(defun spot-kinova (&key (type :gen3_robotiq_2f_140) &rest args &allow-other-keys)
  (setq *spot-kinova* (case type
                       (:gen3_robotiq_2f_85
                        (instance* spot_gen3_robotiq_2f_85-robot :init args))
                       (:gen3_robotiq_2f_140
                        (instance* spot_gen3_robotiq_2f_140-robot :init args))
                       (:gen3_lite_gen3_lite_2f
                        (instance* spot_gen3_lite_gen3_lite_2f-robot :init args))
                       (t
                        (warning-message 1 "unknown kinova robot type ~A~%" type)
                        )))
  *spot-kinova*)

(defmethod spot_gen3_robotiq_2f_85-robot
  (:arm (&rest args) (send* self :head args))      ;; enable to call send *kinova* :arm :angle-vector
  (:go-grasp (&key (pos 0))
    ;; mimic joint
    ;; https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_arg2f_85_model_macro.xacro
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
  (:spot-angle-vector ()
    (coerce (butlast (coerce (send self :angle-vector) cons) 7) float-vector))
  (:kinova-angle-vector ()
    (coerce (nthcdr 12 (coerce (send self :angle-vector) cons)) float-vector))
  )
(defmethod spot_gen3_robotiq_2f_140-robot
  (:arm (&rest args) (send* self :head args))      ;; enable to call send *kinova* :arm :angle-vector
  (:go-grasp (&key (pos 0))
   ;; mimic joint
   ;; https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/grippers/robotiq_2f_140/urdf/robotiq_arg2f_140_model_macro.xacro
    (let ((pos-deg (rad2deg pos)))
      (send self :finger_joint :joint-angle pos-deg)
      (send self :left_inner_finger_joint :joint-angle pos-deg)
      (send self :left_inner_knuckle_joint :joint-angle (* -1 pos-deg))
      (send self :right_inner_finger_joint :joint-angle pos-deg)
      (send self :right_inner_knuckle_joint :joint-angle (* -1 pos-deg))
      (send self :right_outer_knuckle_joint :joint-angle (* -1 pos-deg))))
  (:start-grasp ()
    (send self :go-grasp :pos 0.69))
  (:stop-grasp ()
    (send self :go-grasp :pos 0.0))
  (:spot-angle-vector ()
    (coerce (butlast (coerce (send self :angle-vector) cons) 7) float-vector))
  (:kinova-angle-vector ()
    (coerce (nthcdr 12 (coerce (send self :angle-vector) cons)) float-vector))
  )
(defmethod spot_gen3_lite_gen3_lite_2f-robot
  (:arm (&rest args) (send* self :head args))      ;; enable to call send *kinova* :arm :angle-vector
  (:go-grasp (&key (pos 0))
    ;; mimic joint
    ;; https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/grippers/gen3_lite_2f/urdf/gen3_lite_2f_macro.xacro
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
  (:spot-angle-vector ()
    (coerce (butlast (coerce (send self :angle-vector) cons) 6) float-vector))
  (:kinova-angle-vector ()
    (coerce (nthcdr 12 (coerce (send self :angle-vector) cons)) float-vector))
  )
