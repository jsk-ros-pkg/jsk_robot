(require :spot "package://spoteus/spot.l")

(defmethod spot-robot
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
          (forward-message-to rear_right_knee_jt (cdr args)))))
  )

