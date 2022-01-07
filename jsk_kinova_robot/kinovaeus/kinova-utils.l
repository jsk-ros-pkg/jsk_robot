(require :kinova "package://kinovaeus/kinova.l")

(unless (assoc :init-orig (send gen3_lite_gen3_lite_2f-robot :methods))
  (rplaca (assoc :init (send gen3_lite_gen3_lite_2f-robot :methods)) :init-orig))

(defmethod gen3_lite_gen3_lite_2f-robot
  (:init
   (&rest args)
   (dolist (b (list :_make_instance_kinova_right_finger_dist_link_geom0
                    :_make_instance_kinova_left_finger_dist_link_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.1 0.2 0.9 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send (class self) :methods)))))
     )
    (send* self :init-orig args)))
