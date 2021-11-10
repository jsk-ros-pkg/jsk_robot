;; rotational-dependent-joint


(defclass rotational-dependent-joint
  :super joint
  :slots (axis dependent-joint))

(defmethod rotational-dependent-joint
  (:init (&rest args
          &key ((:axis ax) :z)
               ((:max-joint-velocity mjv) 5) ;; [rad/s]
               ((:max-joint-torque mjt) 100) ;; [Nm]
          &allow-other-keys)
         "create instance of rotational-joint. :axis is either (:x, :y, :z) or vector. :min-angle and :max-angle takes in radius, but velocity and torque are given in SI units."
	 (setq axis ax)
	 (setq joint-angle 0.0)
	 (send-super* :init
                      :max-joint-velocity mjv
                      :max-joint-torque mjt
                      args)
	 ;; set default value
	 (if (null min-angle) (setq min-angle -90.0))
	 (if (null max-angle) (setq max-angle (+ 180.0 min-angle)))
	 (send self :joint-velocity 0.0) ;; [rad/s]
	 (send self :joint-acceleration 0.0) ;; [rad/s^2]
	 (send self :joint-torque 0.0) ;; [Nm]
	 self)
  (:set-dependent-joint ;; set dependent joint
   (other-joint)
   (setq dependent-joint other-joint))
  (:joint-angle-individual ;; use in joint-angle
   (&optional v &key relative &allow-other-keys)
   "Return joint-angle if v is not set, if v is given, set joint angle. v is rotational value in degree."
   (let ()
     (when v
       (when (and joint-min-max-table joint-min-max-target)
         (setq min-angle (send self :joint-min-max-table-min-angle)
               max-angle (send self :joint-min-max-table-max-angle)))
       (if relative (setq v (+ v joint-angle)))
       (cond ((> v max-angle)
	      (unless relative (warning-message 3 ";; ~A :joint-angle(~A) violate max-angle(~A)~%" self v max-angle))
	      (setq v max-angle)))
       (cond ((< v min-angle)
	      (unless relative (warning-message 3 ";; ~A :joint-angle(~A) violate min-angle(~A)~%" self v min-angle))
	      (setq v min-angle)))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :rotate (deg2rad joint-angle) axis))
     joint-angle))
  (:joint-angle ;; apply joint-angle to both self and dependent-joint
   (&optional v &key relative &allow-other-keys)
   (send dependent-joint :joint-angle-individual v :relative relative)
   (send self :joint-angle-individual v :relative relative))
  (:joint-dof () "Returns DOF of rotational joint, 1." 1)
  (:calc-angle-speed-gain (dav i periodic-time) (calc-angle-speed-gain-scalar self dav i periodic-time))
  (:speed-to-angle (v) "Returns degree of given input in radian" (rad2deg v))
  (:angle-to-speed (v) "Returns radian of given input in degree" (deg2rad v))
  (:calc-jacobian (&rest args) (apply #'calc-jacobian-rotational args))
  )
