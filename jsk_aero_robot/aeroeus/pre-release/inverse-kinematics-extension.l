#!/usr/bin/env roseus

;; prerequisites : roslaunch aero_startup aero_reach_controller.launch

(ros::load-ros-manifest "roseus")
(ros::load-ros-package "aero_startup")

(ros::roseus "aero_reach_controller_eusconnect")
(ros::wait-for-service "/larm/aero_reach_controller")
(ros::wait-for-service "/torso_larm/aero_reach_controller")

(setq *tl* (instance ros::transform-listener :init))

(defun ~tf (&optional (arm nil))
  (let (m
	)
    (ros::spin-once)
    (if (eq arm :larm)
	(setq m (send *tl* :lookup-transform "leg_base_link" "object_l"
		      (ros::time)))
      (setq m (send *tl* :lookup-transform "leg_base_link" "object_r"
		    (ros::time)))
      )
    (if (eq m nil) (return-from ~tf nil))
    (if (and (eps= (elt (send m :pos) 0) 0 0.01)
	     (eps= (elt (send m :pos) 1) 0 0.01)
	     (eps= (elt (send m :pos) 2) 0 0.01))
	(return-from ~tf nil)
      )
    (setq m (make-coords :pos (v+ (send m :pos)
				  (send (send (car (send *aero* :links))
					      :copy-worldcoords) :pos))
			 :rot (send m :rot)))
    (send m :draw-on :flush t)
    m))

(defmethod AeroUpperRobot-robot
  (:ik-learned (arm coords &key (torso nil) (revert-if-fail t))
     (let (req
	   res
	   get-coords
	   original-coords
	   )
       (cond ((eq torso nil)
	      (setq req (instance
			 aero_startup::AeroReachControllerServiceSimpleRequest
			 :init))
	      (if (eq arm :rarm)
		  (setq coords
			(make-coords :pos (float-vector
					   (elt (send coords :pos) 0)
					   (- (elt (send coords :pos) 1))
					   (elt (send coords :pos) 2))
				     :rot (send coords :rot)))
		)
	      (send req :x (elt (send coords :pos) 0))
	      (send req :y (elt (send coords :pos) 1))
	      (send req :z (elt (send coords :pos) 2))
	      (if (eq arm :larm)
		  (send req :neck_y (send *aero* :head :neck-y :joint-angle))
		(send req :neck_y (- (send *aero* :head :neck-y :joint-angle)))
		)
	      (setq res (ros::service-call "/larm/aero_reach_controller" req))
	      (setq get-coords
		    (make-coords :pos (v+ (send coords :pos)
					  (v- (send coords :pos)
					      (float-vector (send res :x)
							    (send res :y)
							    (send res :z))))
				 :rot (send coords :rot)))
	      (if (eq arm :rarm)
		  (setq get-coords
			(make-coords :pos (float-vector
					   (elt (send get-coords :pos) 0)
					   (- (elt (send get-coords :pos) 1))
					   (elt (send get-coords :pos) 2))
				     :rot (send coords :rot)))
		)
	      (send *aero* arm :inverse-kinematics get-coords
		    :revert-if-fail revert-if-fail)
	      )
	     ((eq torso t)
	      (setq req (instance
			 aero_startup::AeroReachControllerServiceComplexRequest
			 :init))
	      (setq original-coords coords)
	      (if (eq arm :rarm)
		  (setq coords
			(make-coords :pos (float-vector
					   (elt (send coords :pos) 0)
					   (- (elt (send coords :pos) 1))
					   (elt (send coords :pos) 2))
				     :rot (send coords :rot)))
		)
	      (send req :x (elt (send coords :pos) 0))
	      (send req :y (elt (send coords :pos) 1))
	      (send req :z (elt (send coords :pos) 2))
	      (send *aero* :head :look-at (send coords :pos))
	      (send req :neck_y (send *aero* :head :neck-y :joint-angle))
	      (setq res (ros::service-call
			 "/torso_larm/aero_reach_controller" req))
	      (setq get-coords
		    (make-coords :pos (v+ (send coords :pos)
					  (v- (send coords :pos)
					      (float-vector (send res :x)
							    (send res :y)
							    (send res :z)))
					  )
				 :rot (send coords :rot)))
	      (if (eq arm :rarm)
		  (setq get-coords
			(make-coords :pos (float-vector
					   (elt (send get-coords :pos) 0)
					   (- (elt (send get-coords :pos) 1))
					   (elt (send get-coords :pos) 2))
				     :rot (send coords :rot)))
		)
	      (send *aero* :torso :waist-p :joint-angle (send res :wp))
	      (if (eq arm :larm)
		  (send *aero* :torso :waist-y :joint-angle (send res :wy))
		(send *aero* :torso :waist-y :joint-angle (- (send res :wy)))
		)
	      (send *aero* :head :look-at (send original-coords :pos))
	      (send *aero* arm :inverse-kinematics get-coords
		    :revert-if-fail revert-if-fail)
	      )
	     )
       get-coords))

  (:grasp-object (arm &key (torso t) (revert-if-fail nil)
		      (offset-x 0) (offset-z 50)) ;; magic number for safety
     (let (mt
	   goal
	   )
       (setq goal (~tf arm))
       (if (eq goal nil) (return-from :grasp-object nil))
       ;; solve position
       (send self :ik-learned arm (make-coords :pos (send goal :pos))
	     :torso torso :revert-if-fail revert-if-fail)
       (send *irtviewer* :draw-objects)
       ;; solve rotation
       (setq mt (make-coords :pos (send (send *aero* :larm :end-coords
					      :copy-worldcoords) :pos)
			     :rot (send goal :rot)))
       (send *aero* arm :inverse-kinematics mt
	     :revert-if-fail revert-if-fail)
       (send *irtviewer* :draw-objects)
       ;; move waist for ik misses
       (setq diff (v- (send goal :pos)
		      (send (send *aero* arm :end-coords :copy-worldcoords)
			    :pos)) )
       (setq diff (v+ diff (float-vector offset-x 0 offset-z)))
       (list goal diff) ))

  )
