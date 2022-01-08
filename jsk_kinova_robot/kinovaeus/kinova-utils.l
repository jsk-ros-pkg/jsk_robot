(require :kinova "package://kinovaeus/kinova.l")

;; Reference https://github.com/jsk-ros-pkg/jsk_robot/pull/1263
(unless (assoc :init-orig (send gen3_lite_gen3_lite_2f-robot :methods))
  (rplaca (assoc :init (send gen3_lite_gen3_lite_2f-robot :methods)) :init-orig))

(unless (assoc :init-orig (send gen3_robotiq_2f_140-robot :methods))
  (rplaca (assoc :init (send gen3_robotiq_2f_140-robot :methods)) :init-orig))

(unless (assoc :init-orig (send gen3_robotiq_2f_85-robot :methods))
  (rplaca (assoc :init (send gen3_robotiq_2f_85-robot :methods)) :init-orig))

(defmethod gen3_lite_gen3_lite_2f-robot
  ;; Reference https://github.com/jsk-ros-pkg/jsk_robot/pull/1263
  (:init
   (&rest args) ;; coloring eus model
   (dolist (b (list :_make_instance_kinova_right_finger_dist_link_geom0
                    :_make_instance_kinova_left_finger_dist_link_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.1 0.2 0.9 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send (class self) :methods)))))
     )
   (send* self :init-orig args)))

(defmethod gen3_robotiq_2f_140-robot
  ;; Reference https://github.com/jsk-ros-pkg/jsk_robot/pull/1263
  (:init
   (&rest args) ;; coloring eus model
   (dolist (b (list :_make_instance_kinova_base_link_geom0
                    :_make_instance_kinova_bracelet_link_geom0
                    ))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.1 0.2 0.9 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send (class self) :methods)))))
     )
    (dolist (b (list :_make_instance_kinova_robotiq_arg2f_base_link_geom0
                     :_make_instance_kinova_left_inner_knuckle_geom0
                     :_make_instance_kinova_right_inner_knuckle_geom0
                     :_make_instance_kinova_left_outer_knuckle_geom0
                     :_make_instance_kinova_right_outer_knuckle_geom0
                     :_make_instance_kinova_left_outer_finger_geom0
                     :_make_instance_kinova_right_outer_finger_geom0
                     :_make_instance_kinova_left_inner_finger_geom0
                     :_make_instance_kinova_right_inner_finger_geom0
                     ))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.2 0.2 0.2 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send (class self) :methods)))))
     )
    (send* self :init-orig args)))

(defmethod gen3_robotiq_2f_85-robot
  ;; Reference https://github.com/jsk-ros-pkg/jsk_robot/pull/1263
  (:init
   (&rest args) ;; coloring eus model
   (dolist (b (list :_make_instance_kinova_base_link_geom0
                    :_make_instance_kinova_bracelet_link_geom0
                    ))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.1 0.2 0.9 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send (class self) :methods)))))
     )
    (dolist (b (list :_make_instance_kinova_robotiq_arg2f_base_link_geom0
                     :_make_instance_kinova_left_inner_knuckle_geom0
                     :_make_instance_kinova_right_inner_knuckle_geom0
                     :_make_instance_kinova_left_outer_knuckle_geom0
                     :_make_instance_kinova_right_outer_knuckle_geom0
                     :_make_instance_kinova_left_outer_finger_geom0
                     :_make_instance_kinova_right_outer_finger_geom0
                     :_make_instance_kinova_left_inner_finger_geom0
                     :_make_instance_kinova_right_inner_finger_geom0
                     ))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.2 0.2 0.2 1.0))
                         '(list :diffuse (float-vector 1.0 1.0 1.0 1.0))
                         (assoc b (send (class self) :methods)))))
     )
    (send* self :init-orig args)))
