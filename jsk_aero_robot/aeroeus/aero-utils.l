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
     )
   )
  (:ik-link-list
   (&optional (ik-group :rarm))
   (let (ret)
     (case
      ik-group
      (:rarm
       (setq ret (send self :link-list (send self :rarm :end-coords :parent)
                       (car (send *aero* :rarm)))))
      (:larm
       (setq ret (send self :link-list (send self :larm :end-coords :parent)
                       (car (send *aero* :larm)))))
      (:rarm-upper-body
       (setq ret (send self :link-list (send self :rarm :end-coords :parent)
                       (send self :torso :waist-y :child-link))))
      (:larm-upper-body
       (setq ret (send self :link-list (send self :larm :end-coords :parent)
                       (send self :torso :waist-y :child-link))))
      (:larm-whole-body
       (setq ret (send self :link-list (send self :larm :end-coords :parent)
                       (send self :torso :lifter-ankle :child-link))))
      (:rarm-whole-body
       (setq ret (send self :link-list (send self :rarm :end-coords :parent)
                       (send self :torso :lifter-ankle :child-link))))
      )
     (setq ret (set-difference ret (list l_wrist_center_lk r_wrist_center_lk body_link_lk)))
     ret
     ))
  (:ik-target (&key (name) (arm :larm))
   "return move-target for inverse-kinematics / name (nil :hand :grasp :pick :index :thumb), arm (:larm :rarm)"
   (if name
       (send self :get
             (intern (format nil "EEF-~C-~A" (elt (string arm) 0) (string name)) *keyword-package*))
     (send self arm :end-coords))
   )
  (:ik-group (&key (range :arm) (arm :larm))
   "return ik-group for inverse-kinematics / range (:arm :upper-body :whole-body), arm (:larm :rarm)"
   (if (eq range :arm)
       arm
     (intern (format nil "~A-~A"
                     (string arm) (string range))
             *keyword-package*)))
  ;;
  (:inverse-kinematics
   (target-coords &rest args &key ik-group move-target &allow-other-keys)
   (unless move-target (setq move-target
                             (if (string= "LARM" (subseq (string ik-group) 0 4))
                                 (send self :larm :end-coords)
                               (send self :rarm :end-coords))))
   (send-super* :inverse-kinematics target-coords
                :move-target move-target
                :link-list (send self :ik-link-list ik-group)
                :additional-jacobi
                (list #'(lambda (ll) (send self :calc-jacobian-for-interlocking-joints ll)))
                :additional-vel
                (list #'(lambda (ll) (send self :calc-vel-for-interlocking-joints ll)))
                :move-joints-hook
                #'(lambda () (send self :update-mimic-joints))
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
  )

(defun aero-robot () (setq *aero* (instance aero-lifter-ik :init)))
