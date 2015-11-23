#!/usr/bin/env roseus

;; prerequisites : roslaunch aero_startup hsi_cloud_segmentation.launch
;; which runs, hsi_color_filter and simple_recognition.py

(ros::load-ros-manifest "roseus")

(load "package://aeroeus/aero-interface.l")

(aero-init)

(setq ((send *aero* :head :neck-r) . min-angle) 0)
(setq ((send *aero* :head :neck-r) . max-angle) 0)

(defvar l-hand-center #f(-50 20 0))
(defvar r-hand-center #f(-50 -20 0))

(defvar *tl* (instance ros::transform-listener :init))

(defvar collision-pairs nil)

;;;;;;;;;;;;;;;;;;
(defun create-collision-pairs (arm)
  (let ((arm-links (list (send *aero* arm :shoulder-y :child-link)
			 (send *aero* arm :elbow-p :child-link)
			 (send *aero* arm :wrist-y :child-link)
			 (send *aero* arm :wrist-r :child-link)
			 ))
	(torso-links (list (send *aero* :torso :waist-y :child-link)
			   (send *aero* :torso :waist-r :child-link)
			   (send *aero* :torso :waist-y :parent-link)
			   ))
	tmp-pairs
	)
    (dolist (x arm-links)
      (dolist (y torso-links)
	(push (cons x y) tmp-pairs)
	))
    (setf collision-pairs tmp-pairs)
    ))

;;;;;;;;;;;;;;;;;;
(defun check-collision ()
  (let ((collides nil)
	)
    (dolist (p collision-pairs)
      (if (not (eq (send *aero* :self-collision-check :pairs (list p)) nil))
	  (setq collides t)
	)
      )
    collides))

;;;;;;;;;;;;;;;;;;  
(defun ~tf ()
  (let (m
        )
    (ros::spin-once)
    (setq m (send *tl* :lookup-transform "leg_base_link" "sample"
		  (ros::time)))
    (if (eq m nil) (return-from ~tf nil))
    (if (and (eps= (elt (send m :pos) 0) 0 0.1)
	     (eps= (elt (send m :pos) 1) 0 0.1)
	     (eps= (elt (send m :pos) 2) 0 0.1))
	(return-from ~tf nil)
      )
    (setq m (make-coords :pos (v+ (send m :pos)
				  (send (send (car (send *aero* :links))
					      :copy-worldcoords) :pos))))
    (send m :draw-on :flush t)
    m))


;; (send *aero* :reset-manip-pose)
;; (setq origin (send *aero* :larm :end-coords :copy-worldcoords))
;; (setq limit-search (make-coords
;; 		    :pos (v+ (send origin :pos) #f(1000 0 0))))
;; (send *aero* :larm :inverse-kinematics limit-search
;;       :revert-if-fail nil
;;       :link-list (send *aero* :link-list
;; 		       (send *aero* :larm :end-coords :parent))
;;       ;; :collision-avoidance-link-pair collision-pairs
;;       :rotation-axis nil
;;       ;; :additional-check #'(lambda ()
;;       ;; 			    (let ((dist
;;       ;; 				   (- (elt (send (send *aero* :larm :end-coords
;;       ;; 						       :copy-worldcoords) :pos) 1)
;;       ;; 				      (elt (send limit-search :pos) 1)))
;;       ;; 				  ) (< dist 0.1)))
;;       )

;;;;;;;;;;;;;;;;;;
(defun setup (arm)
  (let (origin
	limit-search
	(result nil)
	)
    (send *aero* :reset-manip-pose)
    (setq origin (send *aero* arm :end-coords :copy-worldcoords))
    ;; z-
    (send *aero* :reset-manip-pose)
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 0 -100))))
    (send *aero* arm :inverse-kinematics limit-search
	  :revert-if-fail nil
	  :link-list (send *aero* :link-list
			   (send *aero* arm :end-coords :parent))
	  :rotation-axis nil
	  )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 2) result) 
    ;; z+
    (send *aero* :reset-manip-pose)
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 0 200))))
    (send *aero* arm :inverse-kinematics limit-search
	  :revert-if-fail nil
	  :link-list (send *aero* :link-list
			   (send *aero* arm :end-coords :parent))
	  :rotation-axis nil
	  )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 2) result)
    ;; y-
    (send *aero* :reset-manip-pose)
    (if (eq arm :larm)
	(setq limit-search (make-coords
			    :pos (v+ (send origin :pos) #f(0 -1000 0))))
      (setq limit-search (make-coords
			  :pos (v+ (send origin :pos) #f(0 -100 0))))
      )
    (send *aero* arm :inverse-kinematics limit-search
	  :revert-if-fail nil
	  :link-list (send *aero* :link-list
			   (send *aero* arm :end-coords :parent))
	  :rotation-axis nil
	  )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 1) result) 
    ;; y+
    (send *aero* :reset-manip-pose)
    (if (eq arm :larm)
	(setq limit-search (make-coords
			    :pos (v+ (send origin :pos) #f(0 100 0))))
      (setq limit-search (make-coords
			  :pos (v+ (send origin :pos) #f(0 1000 0))))
      )
    (send *aero* arm :inverse-kinematics limit-search
	  :revert-if-fail nil
	  :link-list (send *aero* :link-list
			   (send *aero* arm :end-coords :parent))
	  :rotation-axis nil
	  )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 1) result)
    ;; x-
    (send *aero* :reset-manip-pose)
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(-100 0 0))))
    (send *aero* arm :inverse-kinematics limit-search
	  :revert-if-fail nil
	  :link-list (send *aero* :link-list
			   (send *aero* arm :end-coords :parent))
	  :rotation-axis nil
	  )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 0) result) 
    ;; x+
    (send *aero* :reset-manip-pose)
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(1000 0 0))))
    (send *aero* arm :inverse-kinematics limit-search
	  :revert-if-fail nil
	  :link-list (send *aero* :link-list
			   (send *aero* arm :end-coords :parent))
	  :rotation-axis nil
	  )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 0) result)
    result))

;;;;;;;;;;;;;;;;;;
(defun run (arm &key (bits 100.0) (y-bits 20.0))
  (let (arm-limits
	rest-pose
	x-steps y-steps z-steps
	target target-origin
	ik-stat
	(res nil)
	(arm-list-actual nil)
	(arm-list-model nil)
	)
    (cond ((eq arm :larm)
	   (setq arm-limits (setup :larm))
	   (send *aero* :reset-manip-pose)
	   (send *aero* :rarm :shoulder-r :joint-angle -30)
	   (send *aero* :rarm :shoulder-y :joint-angle -90)
	   (setq rest-pose (send *aero* :angle-vector))
	   (create-collision-pairs :larm)
	   )
	  (t
	   (setq arm-limits (setup :rarm))
	   (send *aero* :reset-manip-pose)
	   (send *aero* :larm :shoulder-r :joint-angle 30)
	   (send *aero* :larm :shoulder-y :joint-angle 90)
	   (setq rest-pose (send *aero* :angle-vector))
	   (create-collision-pairs :rarm)
	   )
	  )
    (setq x-steps (/ (- (elt arm-limits 0) (elt arm-limits 1)) bits))
    (setq y-steps (/ (- (elt arm-limits 2) (elt arm-limits 3)) bits))
    (setq z-steps (/ (- (elt arm-limits 4) (elt arm-limits 5)) bits))
    (setq target-origin (make-coords
			 :pos (float-vector (elt arm-limits 1)
					    (elt arm-limits 3)
					    (elt arm-limits 5))))
    (dotimes (j y-steps)
      (dotimes (k z-steps)
	(dotimes (i x-steps)
	  (setq target (make-coords
			:pos
			(v+ (send (send target-origin :copy-worldcoords) :pos)
			    (float-vector (* i bits) (* j bits) (* k bits)))))
	  (send *aero* :angle-vector rest-pose)
	  (setq ik-stat (send *aero* arm :inverse-kinematics target
			      :revert-if-fail nil
			      :link-list (send *aero* :link-list
					       (send *aero*
						     arm :end-coords :parent))))
	  (setq ik-stat (send *aero* arm :inverse-kinematics target
			      :revert-if-fail nil
			      :link-list (send *aero* :link-list
					       (send *aero*
						     arm :end-coords :parent))))
	  (cond ((and (not (check-collision)) ik-stat)
		 (send *aero* :head :look-at
		       (send (send *aero* arm :end-coords :copy-worldcoords)
			     :pos))
		 (send *irtviewer* :draw-objects)
		 (cond ((eq *real* t)
			(print "moving arm to pose")
			(send *ri* :angle-vector
			      (send *aero* :angle-vector) 2000)
			(unix::usleep (* 5000 1000))
			(setq res (~tf))
			(cond ((not (eq res nil))
			       ;; modify here the outputs
			       (push (concatenate float-vector
						  (send res :pos)
						  (float-vector
						   (send *aero*
							 :torso :waist-p :joint-angle)
						   (send *aero*
							 :torso :waist-y :joint-angle)
						   )
						  ) arm-list-actual)
			       (print (format nil "found ~A" (send res :pos)))
			       (print (format nil "torso is ~A ~A"
					      (send *aero* :torso :waist-p :joint-angle)
					      (send *aero* :torso :waist-y :joint-angle)))
			       (let ((tmp-hand (send *aero* arm
						     :end-coords :copy-worldcoords))
				     )
				 (send *aero* :reset-manip-pose)
				 (send *aero* :head :look-at (send tmp-hand :pos))
				 ;; modify here the inputs
				 (push (concatenate float-vector
						    (v+ (send tmp-hand :pos)
							(transform (send tmp-hand :rot)
								   (if (eq arm :larm)
								       l-hand-center
								     r-hand-center))
							)
						    (float-vector
						     (send *aero*
							   :head :neck-y :joint-angle)
						     )
						    ) arm-list-model)
				 )
			       (print (format nil "neck is ~A"
					      (send *aero* :head :neck-y :joint-angle)))
			       )
			      (t
			       (print (format nil "hand not found"))
			       )
			      )
			)
		       (t (unix::usleep (* 1000 1000)))
		       )
		 ))
	  )))
    (list arm-list-model arm-list-actual)))

;;;;;;;;;;;;;;;;;;
(defun save (name data)
  (with-open-file (str (concatenate string name "_in.txt") :direction :output
		       :if-exists :supersede)
		  ;; modify here the number of inputs
		  (dolist (x (car data))
		    (format str "~A ~A ~A ~A~%"
			    (elt x 0) (elt x 1) (elt x 2) (elt x 3))
		    )
		  )
  ;; for each output variable
  (with-open-file (str (concatenate string name "_x_out.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (cadr data))
		    (format str "~A~%" (elt x 0))
		    )
		  )
  (with-open-file (str (concatenate string name "_y_out.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (cadr data))
		    (format str "~A~%" (elt x 1))
		    )
		  )
  (with-open-file (str (concatenate string name "_z_out.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (cadr data))
		    (format str "~A~%" (elt x 2))
		    )
		  )
  (with-open-file (str (concatenate string name "_torso_p_out.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (cadr data))
		    (format str "~A~%" (elt x 3))
		    )
		  )
  (with-open-file (str (concatenate string name "_torso_y_out.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (cadr data))
		    (format str "~A~%" (elt x 4))
		    )
		  )
  )
