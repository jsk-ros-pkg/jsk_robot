#!/usr/bin/env roseus

;; prerequisites : roslaunch aero_startup aero_reach_controller.launch

(ros::load-ros-manifest "roseus")
(ros::load-ros-package "aero_startup")

(ros::roseus "aero_reach_controller_eusconnect")
(ros::wait-for-service "/larm/aero_reach_controller")
;; (ros::wait-for-service "/rarm/aero_reach_controller")

(setq *tl* (instance ros::transform-listener :init))

(defun ~tf (&optional (arm nil))
  (let (m
	z
	target-p
	target-m
	)
    (ros::spin-once)
    (setq m (send *tl* :lookup-transform "leg_base_link" "object" (ros::time)))
    (if (eq m nil) (return-from ~tf nil))

    (setq target-p (v+ (send m :pos)
		       (send (send (car (send *aero* :links))
				   :copy-worldcoords) :pos)))

    (setq target-m (m* (send m :rot)
		       (rotation-matrix (/ pi 2) #f(0 1 0))
		       (inverse-matrix (send m :rot))))
    (setq z (transform target-m #f(0 0 1)))
    (cond ((eq arm :larm)
	   (if (< (v. z #f(-1 -1 1)) (v. z #f(1 1 -1)) )
	       (setq target-m (m* target-m
				  (rotation-matrix pi #f(1 0 0))
				  (inverse-matrix target-m)))
	     )
	   )
	  ((eq arm :rarm)
	   (if (< (v. z #f(-1 1 1)) (v. z #f(1 -1 -1)) )
	       (setq target-m (m* target-m
				  (rotation-matrix pi #f(1 0 0))
				  (inverse-matrix target-m)))
	     )
	   )
	  )

    (setq m (make-coords :pos target-p :rot target-m))
    (send m :draw-on :flush t)
    m))

(defmethod AeroUpperRobot-robot
  (:ik-x (arm coords &key (torso nil) (revert-if-fail nil))
     (let (req
	   res
	   get-coords
	   )
       (setq req (instance aero_startup::AeroReachControllerServiceRequest
			   :init))
       (send req :x (elt (send coords :pos) 0))
       (send req :y (elt (send coords :pos) 1))
       (send req :z (elt (send coords :pos) 2))
       (send req :neck_y (send *aero* :head :neck-y :joint-angle))
       (if (eq arm :larm)
	   (setq res (ros::service-call "/larm/aero_reach_controller" req))
	 (setq res (ros::service-call "/rarm/aero_reach_controller" req))
	 )
       (setq get-coords
	     (make-coords :pos (v+ (send coords :pos)
				   (v- (send coords :pos)
				       (float-vector (send res :x)
						     (send res :y)
						     (send res :z))))
			  :rot (send coords :rot)))
       (send *aero* arm :inverse-kinematics get-coords
	     :revert-if-fail revert-if-fail)
       get-coords))

  (:grasp-object (arm &key (torso nil) (revert-if-fail nil))
     (let (mt
	   )
       (send *aero* :ungrasp arm)
       ;; solve position
       (send self :ik-x arm (make-coords :pos (send (~tf arm) :pos))
	     :torso torso :revert-if-fail revert-if-fail)
       (send *irtviewer* :draw-objects)
       ;; solve rotation
       (setq mt (make-coords :pos (send (send *aero* :larm :end-coords
					      :copy-worldcoords) :pos)
			     :rot (send (~tf arm) :rot)))
       (send *aero* arm :inverse-kinematics mt
	     :revert-if-fail revert-if-fail)
       mt))

  )
