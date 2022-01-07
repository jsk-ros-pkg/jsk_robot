(require :spotkinova "package://spotkinovaeus/spotkinova.l")

(unless (assoc :init-orig (send spot_gen3_lite_gen3_lite_2f-robot :methods))
  (rplaca (assoc :init (send spot_gen3_lite_gen3_lite_2f-robot :methods)) :init-orig))

(defmethod spot_gen3_lite_gen3_lite_2f-robot
  (:init
   (&rest args) ;; fix colors
   (dolist (b (list :_make_instance_body_geom0
                    :_make_instance_front_left_upper_leg_geom0
                    :_make_instance_front_left_lower_leg_geom0
                    :_make_instance_front_right_upper_leg_geom0
                    :_make_instance_front_right_lower_leg_geom0
                    :_make_instance_rear_left_upper_leg_geom0
                    :_make_instance_rear_left_lower_leg_geom0
                    :_make_instance_rear_right_upper_leg_geom0
                    :_make_instance_rear_right_lower_leg_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(1.00 0.84 0.32 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send  (class self) :methods)))))
     )
   (dolist (b (list :_make_instance_front_left_hip_geom0
                    :_make_instance_front_right_hip_geom0
                    :_make_instance_rear_left_hip_geom0
                    :_make_instance_rear_right_hip_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.1 0.1 0.1 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send  (class self) :methods)))))
     )
      (send* self :init-orig args))
  (:legs ;; support legs for all limbs
   (&rest args)
   (case (car args)
         (:crotch-r
          (forward-message-to front_left_hip_x_jt (cdr args))
          (forward-message-to front_right_hip_x_jt (cdr args))
          (forward-message-to rear_left_hip_x_jt (cdr args))
          (forward-message-to rear_right_hip_x_jt (cdr args)))
         (:crotch-p
          (forward-message-to front_left_hip_y_jt (cdr args))
          (forward-message-to front_right_hip_y_jt (cdr args))
          (forward-message-to rear_left_hip_y_jt (cdr args))
          (forward-message-to rear_right_hip_y_jt (cdr args)))
         (:knee-p
          (forward-message-to front_left_knee_jt (cdr args))
          (forward-message-to front_right_knee_jt (cdr args))
          (forward-message-to rear_left_knee_jt (cdr args))
          (forward-message-to rear_right_knee_jt (cdr args))))))
