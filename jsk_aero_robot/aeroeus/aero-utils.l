(setq *aero-robot-class* nil)

(dolist (cls (sys::list-all-classes))
  (let ((str (string (send cls :name))))
    (when (and (> (length str) 10)
               (string= (subseq str 0 4) "AERO")
               (string= (subseq str (- (length str) 6)) "-ROBOT")
               )
      (warn ";; aero robot: ~A~%" (subseq str 4 (- (length str) 6)))
      (setq *aero-robot-class* cls)
      (return)
      )
    ))

(unless *aero-robot-class*
  (warn ";; can not find class for aero robot~%")
  )

(defclass aero-lifter-ik
  :super *aero-robot-class*
  :slots ())
(defmethod aero-lifter-ik
  (:init
   (&rest args)
   (prog1
       (send-super* :init args)
     ;; generate grasp coords
     (let ((l-p (send self :larm :end-coords :parent))
           (r-p (send self :rarm :end-coords :parent))
           )
       (dolist (pair (list (cons :eef-l-hand  :l_hand_link_lk)
                           (cons :eef-l-grasp :l_eef_grasp_link_lk)
                           (cons :eef-l-pick  :l_eef_pick_link_lk)
                           (cons :eef-l-index :l_index_tip_link_lk)
                           (cons :eef-l-thumb :l_thumb_tip_link_lk)))
         (send self :put (car pair)
               (let ((cds (make-cascoords
                           :coords (send self (cdr pair) :copy-worldcoords))))
                 (send cds :name (car pair))
                 (send l-p :assoc cds)
                 cds))
         )
       (dolist (pair (list (cons :eef-r-hand  :r_hand_link_lk)
                           (cons :eef-r-grasp :r_eef_grasp_link_lk)
                           (cons :eef-r-pick  :r_eef_pick_link_lk)
                           (cons :eef-r-index :r_index_tip_link_lk)
                           (cons :eef-r-thumb :r_thumb_tip_link_lk)))
         (send self :put (car pair)
               (let ((cds (make-cascoords
                           :coords (send self (cdr pair) :copy-worldcoords))))
                 (send cds :name (car pair))
                 (send r-p :assoc cds)
                 cds))
         )
       )
     (dolist (j (list (send self :ankle_joint_mimic)
                      (send self :knee_joint_mimic)
                      (send self :ankle_joint_dummy)
                      (send self :knee_joint_dummy)))
       (send j :max-angle 180)
       (send j :min-angle -180))
     )
   )
  (:ik-link-list
   (&optional (ik-group :rarm) (link-list))
   (let (ret)
     (case
      ik-group
      (:rarm
       (setq ret (send self :link-list (send self :rarm :end-coords :parent)
                       (car (send self :rarm)))))
      (:larm
       (setq ret (send self :link-list (send self :larm :end-coords :parent)
                       (car (send self :larm)))))
      (:rarm-upper-body
       (setq ret (send self :link-list (send self :rarm :end-coords :parent)
                       (send self :torso :waist-y :child-link))))
      (:larm-upper-body
       (setq ret (send self :link-list (send self :larm :end-coords :parent)
                       (send self :torso :waist-y :child-link))))
      (:rarm-whole-body
       (setq ret (send self :link-list (send self :rarm :end-coords :parent)
                       (send self :torso :lifter-ankle :child-link))))
      (:larm-whole-body
       (setq ret (send self :link-list (send self :larm :end-coords :parent)
                       (send self :torso :lifter-ankle :child-link))))
      (:both-arm
       (setq ret (list (send self :ik-link-list :rarm)
                       (send self :ik-link-list :larm))))
      (:both-arm-upper-body
       (setq ret (list (send self :ik-link-list :rarm-upper-body)
                       (send self :ik-link-list :larm-upper-body))))
      (:both-arm-whole-body
       (setq ret (list (send self :ik-link-list :rarm-whole-body)
                       (send self :ik-link-list :larm-whole-body))))
      (nil
       (setq ret link-list))
      )
     (unless ret
       (warn ";; invalid ik-group ~A~%" ik-group)
       (return-from :ik-link-list))
     ;;
     (setq ret (set-difference ret (list l_wrist_center_lk r_wrist_center_lk body_link_lk)))
     ret
     ))
  (:ik-target (&key (name) (arm :larm))
   "return move-target for inverse-kinematics / name (nil :hand :grasp :pick :index :thumb), arm (:larm :rarm)"
   (cond
    ((string= "BOTH" (subseq (string arm) 0 4))
     (list (send self :ik-target :name name :arm :rarm)
           (send self :ik-target :name name :arm :larm))
     )
    (name
     (send self :get
           (intern (format nil "EEF-~C-~A" (elt (string arm) 0) (string name)) *keyword-package*)))
    (t (send self arm :end-coords))
    )
   )
  (:ik-group (&key (range :arm) (arm :larm))
   "return ik-group for inverse-kinematics / range (:arm :upper-body :whole-body), arm (:larm :rarm :both-arm)"
   (if (eq range :arm)
       arm
     (intern (format nil "~A-~A"
                     (string arm) (string range))
             *keyword-package*))
   )
  ;; ik-group :rarm :larm :both-arm :rarm-upper-body :larm-upper-body :both-arm-upper-body
  ;;          :rarm-whole-body :larm-whole-body :both-arm-whole-body
  (:inverse-kinematics
   (target-coords &rest args &key ik-group move-target &allow-other-keys)
   (unless move-target (setq move-target
                             (cond
                              ((string= "BOTH" (subseq (string ik-group) 0 4))
                               (list (send self :rarm :end-coords)
                                     (send self :larm :end-coords)))
                              ((string= "LARM" (subseq (string ik-group) 0 4))
                               (send self :larm :end-coords))
                              (t
                               (send self :rarm :end-coords)))
                             ))
   ;; check
   (unless
       (or (and (atom target-coords) (atom move-target))
           (and (not (or (atom target-coords) (atom move-target)))
                (= (length target-coords) (length move-target))))
     (warn ";; length mismatch :target-coords ~A :move-target ~A~%" target-coords move-target)
     (return-from :inverse-kinematics nil))
   (send-super* :inverse-kinematics target-coords
                :move-target move-target
                :link-list (send self :ik-link-list ik-group (cadr (memq :link-list args)))
                :additional-jacobi
                (list #'(lambda (ll) (send self :calc-jacobian-for-interlocking-joints ll)))
                :additional-vel
                (list #'(lambda (ll) (send self :calc-vel-for-interlocking-joints ll)))
                :move-joints-hook
                #'(lambda () (send self :update-mimic-joints))
                args)
   )
  ;; compatibility with aero_std
  (:set-from-IK
   (target-coords &rest args &key (arm :larm) (range :arm) (end-effector) &allow-other-keys)
   (send* self :inverse-kinematics target-coords
          :ik-group (send self :ik-group :arm arm :range range)
          :move-target (send self :ik-target :arm arm :name end-effector)
          args)
   )
  ;;
  (:calc-jacobian-for-interlocking-joints
   (link-list &key (interlocking-joint-pairs (send self :interlocking-joint-pairs)))
   "Calculate jacobian to keep interlocking joint velocity --same-- opposite.
    d\theta_0 = d\theta_1 => [... 0 1 0 ... 0  -1 0 .... ][...d\theta_0...d\theta_1...]^t = [0] => d\theta_0 - d\theta_1 = 0"
   (let* ((union-link-list (send self :calc-union-link-list link-list))
          (jl (send-all union-link-list :joint))
          (pairs (remove-if-not #'(lambda (pair) (and (find (car pair) jl) (find (cdr pair) jl))) interlocking-joint-pairs))
          (jacobi (make-matrix (length pairs) (send self :calc-target-joint-dimension union-link-list))))
     (mapcar #'(lambda (pair)
                 (let ((idx (reduce #'+ (send-all (subseq jl 0 (position (car pair) jl)) :joint-dof))))
                   (setf (aref jacobi (position pair pairs) idx) 1.0))
                 (let ((idx (reduce #'+ (send-all (subseq jl 0 (position (cdr pair) jl)) :joint-dof))))
                   (setf (aref jacobi (position pair pairs) idx) 1.0)))
             pairs)
     jacobi))
  ;; just copy from irtmodel
  (:calc-vel-for-interlocking-joints
   (link-list &key (interlocking-joint-pairs (send self :interlocking-joint-pairs)))
   "Calculate 0 velocity for keeping interlocking joint at the same joint angle."
   (let* ((jl (send-all (send self :calc-union-link-list link-list) :joint))
          (pairs (remove-if-not #'(lambda (pair) (and (find (car pair) jl) (find (cdr pair) jl))) interlocking-joint-pairs))
          (vel (instantiate float-vector (length pairs))))
     vel))
  (:update-mimic-joints ()
   (send ankle_joint_mimic_jt :joint-angle
         (- (send ankle_joint_jt :joint-angle)))
   (send knee_joint_mimic_jt :joint-angle
         (- (send knee_joint_jt :joint-angle)))
   (send ankle_joint_dummy_jt :joint-angle
         (send ankle_joint_jt :joint-angle))
   (send knee_joint_dummy_jt :joint-angle
         (- (send knee_joint_jt :joint-angle)))
   t
   )
  (:interlocking-joint-pairs ()
   (list (cons ankle_joint_jt
               ankle_joint_mimic_jt)
         (cons knee_joint_jt
               knee_joint_mimic_jt)
         ))
  (:angle-vector (&rest args)
   (prog1
       (send-super* :angle-vector args)
     (send self :update-mimic-joints))
   )
  )

(defun aero-robot () (setq *aero* (instance aero-lifter-ik :init)))
