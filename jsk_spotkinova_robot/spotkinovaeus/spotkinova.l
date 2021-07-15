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
  (:fullbody-inverse-kinematics
    (target-coords
     &rest args
     &key (move-target) (link-list)
          (min (float-vector -500 -500  -500 -20 -20 -10))
          (max (float-vector  500  500   500  20  20  10))
          (target-centroid-pos)
          (rotation-axis)
     &allow-other-keys)
    "The purpose of this function is to use :fullbody-inverse-kinematics by only specifying :head :end-coords target-coords.
Example:
(send *spot-kinova* :fullbody-inverse-kinematics (make-coords :pos #f(1200 0 800)))
"
    ;; If target-coords is coordinate class, it is passed to :head
    (unless (listp target-coords)
      (setq target-coords
            (list target-coords
                  (send self :larm :end-coords :copy-worldcoords)
                  (send self :rarm :end-coords :copy-worldcoords)
                  (send self :lleg :end-coords :copy-worldcoords)
                  (send self :rleg :end-coords :copy-worldcoords))))
    (unless move-target
      (setq move-target
            (list (send self :head :end-coords)
                  (send self :larm :end-coords)
                  (send self :rarm :end-coords)
                  (send self :lleg :end-coords)
                  (send self :rleg :end-coords))))
    (unless link-list
      (setq link-list
            (mapcar #'(lambda (limb) (send self :link-list (send limb :parent))) move-target)))
    (unless rotation-axis
      (setq rotation-axis (list t nil nil nil nil)))
    (send-super*
          :fullbody-inverse-kinematics target-coords
          :move-target move-target
          :link-list link-list
          :min min
          :max max
          :target-centroid-pos target-centroid-pos
          :rotation-axis rotation-axis
          args))
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
  (:fullbody-inverse-kinematics
    (target-coords
     &rest args
     &key (move-target) (link-list)
          (min (float-vector -500 -500  -500 -20 -20 -10))
          (max (float-vector  500  500   500  20  20  10))
          (target-centroid-pos)
          (rotation-axis)
      &allow-other-keys)
    "The purpose of this function is to use :fullbody-inverse-kinematics by only specifying :head :end-coords target-coords.
Example:
(send *spot-kinova* :fullbody-inverse-kinematics (make-coords :pos #f(1200 0 800)))
"
    ;; If target-coords is coordinate class, it is passed to :head
    (unless (listp target-coords)
      (setq target-coords
            (list target-coords
                  (send self :larm :end-coords :copy-worldcoords)
                  (send self :rarm :end-coords :copy-worldcoords)
                  (send self :lleg :end-coords :copy-worldcoords)
                  (send self :rleg :end-coords :copy-worldcoords))))
    (unless move-target
      (setq move-target
            (list (send self :head :end-coords)
                  (send self :larm :end-coords)
                  (send self :rarm :end-coords)
                  (send self :lleg :end-coords)
                  (send self :rleg :end-coords))))
    (unless link-list
      (setq link-list
            (mapcar #'(lambda (limb) (send self :link-list (send limb :parent))) move-target)))
    (unless rotation-axis
      (setq rotation-axis (list t nil nil nil nil)))
    (send-super*
          :fullbody-inverse-kinematics target-coords
          :move-target move-target
          :link-list link-list
          :min min
          :max max
          :target-centroid-pos target-centroid-pos
          :rotation-axis rotation-axis
          args))
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
  (:fullbody-inverse-kinematics
    (target-coords
     &rest args
     &key (move-target) (link-list)
          (min (float-vector -500 -500  -500 -20 -20 -10))
          (max (float-vector  500  500   500  20  20  10))
          (target-centroid-pos)
          (rotation-axis)
      &allow-other-keys)
    "The purpose of this function is to use :fullbody-inverse-kinematics by only specifying :head :end-coords target-coords.
Example:
(send *spot-kinova* :fullbody-inverse-kinematics (make-coords :pos #f(1200 0 800)))
"
    ;; If target-coords is coordinate class, it is passed to :head
    (unless (listp target-coords)
      (setq target-coords
            (list target-coords
                  (send self :larm :end-coords :copy-worldcoords)
                  (send self :rarm :end-coords :copy-worldcoords)
                  (send self :lleg :end-coords :copy-worldcoords)
                  (send self :rleg :end-coords :copy-worldcoords))))
    (unless move-target
      (setq move-target
            (list (send self :head :end-coords)
                  (send self :larm :end-coords)
                  (send self :rarm :end-coords)
                  (send self :lleg :end-coords)
                  (send self :rleg :end-coords))))
    (unless link-list
      (setq link-list
            (mapcar #'(lambda (limb) (send self :link-list (send limb :parent))) move-target)))
    (unless rotation-axis
      (setq rotation-axis (list t nil nil nil nil)))
    (send-super*
          :fullbody-inverse-kinematics target-coords
          :move-target move-target
          :link-list link-list
          :min min
          :max max
          :target-centroid-pos target-centroid-pos
          :rotation-axis rotation-axis
          args))
  )
