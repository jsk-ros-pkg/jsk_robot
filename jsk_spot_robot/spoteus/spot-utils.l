(require :spot "package://spoteus/spot.l")

(unless (assoc :init-orig (send spot-robot :methods))
  (rplaca (assoc :init (send spot-robot :methods)) :init-orig))

(defmethod spot-robot
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
             (cdr (subst '(list :diffuse #f(1.00 0.84 0.32 0))
                         '(list :diffuse (float-vector 1.000000 1.000000 1.000000 1.000000))
                         (assoc b (send  (class self) :methods)))))
     )
   (dolist (b (list :_make_instance_arm0.link_sh0_geom0
                    :_make_instance_arm0.link_sh1_geom0
                    :_make_instance_arm0.link_hr0_geom0
;                    :_make_instance_arm0.link_el0_geom0
                    :_make_instance_arm0.link_el1_geom0
                    :_make_instance_arm0.link_wr0_geom0
                    :_make_instance_arm0.link_wr1_geom0
                    :_make_instance_arm0.link_fngr_geom0
                    ))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(1.00 0.84 0.32 0))
                         '(list :diffuse (float-vector 0.200000 0.200000 0.200000 1.000000))
                         (assoc b (send  (class self) :methods)))))
     )
   (dolist (b (list :_make_instance_front_left_hip_geom0
                    :_make_instance_front_right_hip_geom0
                    :_make_instance_rear_left_hip_geom0
                    :_make_instance_rear_right_hip_geom0))
     (rplacd (assoc b (send (class self) :methods))
             (cdr (subst '(list :diffuse #f(0.1 0.1 0.1 0))
                         '(list :diffuse (float-vector 1.000000 1.000000 1.000000 1.000000))
                         (assoc b (send  (class self) :methods)))))
     )
   (prog1
     (send* self :init-orig args)
     ;; add head-end-coords (camera coords)
     (let ((head-end-coords (make-cascoords :coords (send arm-end-coords :copy-worldcoords))))
       (send head-end-coords :rotate pi/2 :y)
       (send (send arm-end-coords :parent) :assoc head-end-coords)
       (send self :put :head-end-coords head-end-coords))))
  ;; add head method so that (send *spot* :head :look-at) works
  (:head
   (&rest args)
   (unless args (setq args (list nil)))
   (case (car args)
	 (:end-coords
	  (user::forward-message-to (send self :get :head-end-coords) (cdr args)))
	 (:look-at
	  (let ((target-coords
		 (orient-coords-to-axis ;; orient target-coords to look-at direction
		  (make-coords :pos (cadr args))
		  (v- (cadr args) (send self :head :end-coords :worldpos))))
		(head-coords (send self :head :end-coords :copy-worldcoords)))
	    ;; align target-coords x (image-y) direction to head direction
	    (send target-coords :rotate
		  (acos (v. (send target-coords :axis :x)
			    (send head-coords :axis :x))) :z)
	    (send* self :head :inverse-kinematics target-coords
		   :move-target (send self :head :end-coords)
		   :rotation-axis t
		   :translation-axis nil
		   :target-coords target-coords ;; for debug-view
		   (cddr args))))
	  ;;(send* self :inverse-kinematics-loop-for-look-at :head (cdr args)))
	 (t
	  (send* self :limb :arm args))))
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
  (:body-inverse-kinematics
    (target-coords
     &rest args
     &key (move-target) (link-list)
          (min (float-vector -500 -500  -500 -200 -200 -100))
          (max (float-vector  500  500   500  200  200  100))
          (target-centroid-pos)
          (rotation-axis)
          (root-link-virtual-joint-weight #f(0.0 0.0 0.1 0.1 0.5 0.5))
     &allow-other-keys)
    "The purpose of this function is to use :fullbody-inverse-kinematics by only specifying body target-coords.
Example:
(send *spot* :body-inverse-kinematics (make-coords :pos #f(0 0 50)))
"
    (let ((body-coords (make-cascoords :coords (send (send self :coords) :copy-worldcoords))))
      (send body_lk :assoc body-coords)
      (setq target-coords
            (list target-coords
                  (send self :larm :end-coords :copy-worldcoords)
                  (send self :rarm :end-coords :copy-worldcoords)
                  (send self :lleg :end-coords :copy-worldcoords)
                  (send self :rleg :end-coords :copy-worldcoords)))
      (unless move-target
        (setq move-target
              (list body-coords
                    (send self :larm :end-coords)
                    (send self :rarm :end-coords)
                    (send self :lleg :end-coords)
                    (send self :rleg :end-coords))))
      (unless link-list
        (setq link-list
              (mapcar #'(lambda (limb) (send self :link-list (send limb :parent))) move-target)))
      (unless rotation-axis
        (setq rotation-axis (list t nil nil nil nil)))
      (prog1
        (send-super*
            :fullbody-inverse-kinematics target-coords
            :move-target move-target
            :link-list link-list
            :min min
            :max max
            :target-centroid-pos target-centroid-pos
            :rotation-axis rotation-axis
            :root-link-virtual-joint-weight root-link-virtual-joint-weight
            args)
        (send body_lk :dissoc body-coords))))
  )
