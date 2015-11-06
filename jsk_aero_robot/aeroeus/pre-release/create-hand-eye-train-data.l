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

(defun ~tf (&optional (arm nil))
  (let (m
        )
    (ros::spin-once)
    (setq m (send *tl* :lookup-transform "leg_base_link" "sample"
		  (ros::time)))
    (if (eq m nil) (return-from ~tf nil))
    (setq m (make-coords :pos (v+ (send m :pos)
				  (send (send (car (send *aero* :links))
					      :copy-worldcoords) :pos))))
    (send m :draw-on :flush t)
    m))

(defun setup (arm &key (torso nil))
  (let (origin
	limit-search
	(result nil)
	)
    (send *aero* :reset-manip-pose)
    (setq origin (send *aero* arm :end-coords :copy-worldcoords))
    ;; z-
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 0 -10000))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 2) result) 
    ;; z+
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 0 10000))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 2) result)
    ;; y-
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 -10000 0))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 1) result) 
    ;; y+
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 10000 0))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 1) result)
    ;; x-
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(-10000 0 0))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 0) result) 
    ;; x+
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(10000 0 0))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 0) result)
    result))


(defun run (arm &key (torso nil) (bits 100.0) (y-bits 20.0))
  (let (arm-limits
	x-steps y-steps z-steps
	target target-origin
	head-p head-y
	(res nil)
	(arm-list-actual nil)
	(arm-list-model nil)
	)
    (setq head-p (send *aero* :head :neck-p :joint-angle))
    (setq head-y (send *aero* :head :neck-y :joint-angle))
    (cond ((eq arm :larm)
	   (setq arm-limits (setup :larm :torso torso))
	   (send *aero* :rarm :shoulder-r :joint-angle -30)
	   (send *aero* :rarm :shoulder-y :joint-angle -90)
	   )
	  (t
	   (setq arm-limits (setup :rarm :torso torso))
	   (send *aero* :larm :shoulder-r :joint-angle 30)
	   (send *aero* :larm :shoulder-y :joint-angle 90)
	   )
	  )
    (send *aero* :head :neck-p :joint-angle head-p)
    (send *aero* :head :neck-y :joint-angle head-y)
    (setq x-steps (/ (- (elt arm-limits 0) (elt arm-limits 1)) bits))
    (setq y-steps (/ (- (elt arm-limits 2) (elt arm-limits 3)) bits))
    (setq z-steps (/ (- (elt arm-limits 4) (elt arm-limits 5)) bits))
    (setq target-origin (make-coords
			 :pos (float-vector (elt arm-limits 1)
					    (elt arm-limits 3)
					    (elt arm-limits 5))))
    (setq target (send target-origin :copy-worldcoords))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics target
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics target
	    :revert-if-fail nil)
      )
    (cond ((eq *real* t)
	   (send *ri* :angle-vector
	   	 (send *aero* :stroke-vector) 2000)
	   (unix::usleep (* 4000 1000))
	   )
	  )
    (send *irtviewer* :draw-objects)
    (dotimes (j y-steps)
      (dotimes (k z-steps)
	(dotimes (i x-steps)
	  (setq target (make-coords
			:pos
			(v+ (send (send target-origin :copy-worldcoords) :pos)
			    (float-vector (* i bits) (* j bits) (* k bits))))) 
	  (if (eq torso t)
	      (send *aero* arm :inverse-kinematics target
		    :revert-if-fail nil
		    :link-list (send *aero* :link-list
				     (send *aero*
					   arm :end-coords :parent)))
	    (send *aero* arm :inverse-kinematics target
		  :revert-if-fail nil)
	    )
	  (send *aero* :head :look-at
		(send (send *aero* arm :end-coords :copy-worldcoords)
		      :pos))
	  (send *irtviewer* :draw-objects)
	  (cond ((eq *real* t)
		 (let ((n-1-found (send (make-coords) :pos))
		       (n-found nil)
		       (i-saw-ghost nil)
		       )
		   (print "moving arm to pose")
		   (send *ri* :angle-vector
			 (send *aero* :stroke-vector) 2000)
		   (unix::usleep (* 5000 1000))
		   (setq res (~tf))
		   (block find-hand
		     (cond ((not (eq res nil))

			    ;; check if found hand is a delay capture or not
			    (setq n-found (send res :pos))
			    (if (and
				 (eps= (elt n-found 0) (elt n-1-found 0) 1.0)
				 (eps= (elt n-found 1) (elt n-1-found 1) 1.0)
				 (eps= (elt n-found 2) (elt n-1-found 2) 1.0))
				(progn (print "I see ghost")
				       (cond ((eq i-saw-ghost nil)
					      (pop arm-list-model)
					      (pop arm-list-actual)
					      (setq i-saw-ghost t)
					      ))
				       (return-from find-hand))
			      )

			    (let ((tmp-hand (send *aero* arm
						  :end-coords
						  :copy-worldcoords))
				  )

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
					) arm-list-model))
			    ;; modify here the inputs

			    ;; modify here the outputs
			    (push (send res :pos) arm-list-actual)
			    ;; modify here the outputs

			    (print (format nil "found ~A" n-found))
			    (setq n-1-found n-found)
			    (setq i-saw-ghost nil)
			    t)
			   )
		     ) ;; block
		 ) ;; let *real*
		) ;; cond *real*
	  )))
    (list arm-list-model arm-list-actual)))

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
  )
