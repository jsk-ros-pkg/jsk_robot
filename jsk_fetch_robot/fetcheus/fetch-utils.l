;;
;;
;;
(require :fetch "package://fetcheus/fetch.l")

(defmethod fetch-robot
  (:inverse-kinematics
   (target-coords &rest args &key
		  link-list move-arm (use-torso t) move-target
                  (use-base nil) (start-coords (send self :copy-worldcoords))
		  (base-range (list :min #f(-30 -30 -30)
				    :max #f( 30  30  30)))
		  additional-weight-list
		  &allow-other-keys)
  (let (diff-pos-rot)
    (unless move-arm (setq move-arm :rarm))
    (unless move-target (setq move-target (send self :rarm :end-coords)))
    (unless link-list
      (setq link-list (send self :link-list (send move-target :parent)
			    (unless use-torso (car (send self :rarm))))))

    (cond
      (use-base
       (setq diff-pos-rot
             (concatenate float-vector
                          (send start-coords :difference-position self)
                          (send start-coords :difference-rotation self)))
       (send self :move-to start-coords :world)
       (with-append-root-joint
	   (ll self link-list
	       :joint-class omniwheel-joint
	       :joint-args base-range)
	 (send (caar ll) :joint :joint-angle
	       (float-vector (elt diff-pos-rot 0)
			     (elt diff-pos-rot 1)
			     (rad2deg (elt diff-pos-rot 5))))
	 (send-super* :inverse-kinematics target-coords
		      :additional-weight-list
		      (append
		       (list
			(list (send self :torso_lift_joint :child-link)
			      (if (numberp use-torso) use-torso 0.005))
			(list (car (send self :links))
			      (if (eq use-base t) 0.1 use-base))
			)
		       additional-weight-list
		       )
		      :link-list ll ;; link-list
		      :move-target move-target
		      args)))
      (t
       (send-super* :inverse-kinematics target-coords
                    :additional-weight-list
                    (list
                     (list (send self :torso_lift_joint :child-link)
                           (if (numberp use-torso) use-torso 0.005))
                     )
                    :link-list link-list
                    :move-target move-target
                    args))))
    ))
