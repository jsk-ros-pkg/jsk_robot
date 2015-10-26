;; methods that will be deprecated in future version

(require :aero-lower "package://aero_description/models/aero-lower.l")
(load "package://aeroeus/aero-interface.l")



(defmethod AeroUpperRobot-robot
  (:collision-check-all ()
     (let* ((upper-links (list (send self :larm :shoulder-y :child-link)
			       (send self :larm :elbow-p :child-link)
			       (send self :larm :wrist-y :child-link)
			       (send self :larm :wrist-r :child-link)
			       (send self :larm :t-1p :child-link)
			       (send self :larm :f-1p :child-link)
			       (send self :larm :f-2p :child-link)
			       (send self :larm :f-3p :child-link)
			       (send self :rarm :shoulder-y :child-link)
			       (send self :rarm :elbow-p :child-link)
			       (send self :rarm :wrist-y :child-link)
			       (send self :rarm :wrist-r :child-link)
			       (send self :rarm :t-1p :child-link)
			       (send self :rarm :f-1p :child-link)
			       (send self :rarm :f-2p :child-link)
			       (send self :rarm :f-3p :child-link)))
	    (lower-links  nil)
	    (check-upper nil)
	    (check-lower nil)
	    (result nil)
	    )
       (setq check-upper (send self :self-collision-check))
       (if (not (eq check-upper nil)) (print check-upper))
       (cond ((boundp '*aero-lower*)
	      (setq lower-links (list (elt (send *aero-lower* :torso :links) 0)
				      (send *aero-lower* :larm :shoulder-y :child-link)
				      (send *aero-lower* :larm :shoulder-p :child-link)
				      (send *aero-lower* :larm :elbow-p :child-link)
				      (send *aero-lower* :lleg :crotch-y :child-link)
				      (send *aero-lower* :lleg :crotch-p :child-link)
				      (send *aero-lower* :lleg :knee-p :child-link)
				      (send *aero-lower* :rarm :shoulder-y :child-link)
				      (send *aero-lower* :rarm :shoulder-p :child-link)
				      (send *aero-lower* :rarm :elbow-p :child-link)
				      (send *aero-lower* :rleg :crotch-y :child-link)
				      (send *aero-lower* :rleg :crotch-p :child-link)
				      (send *aero-lower* :rleg :knee-p :child-link)))
	      (dolist (x upper-links)
		(dolist (y lower-links)
		  (setq result (send self :self-collision-check :pairs (list (cons x y))))
		  (if (not (eq result nil))
		      (progn (print result)
			     (setq check-lower t)))
		  ))
	      ))
       (if (and (eq check-upper nil)
		(eq check-lower nil))
	   t
	 nil)
       )
     )
  )



(defclass AeroRobot-robot
  :super object
  :slots ())
(defmethod AeroRobot-robot


  ;;;;;;;;;;;;;;;;;; init ;;;;;;;;;;;;;;;;;;
  (:init ()
    (if (not (boundp `*aero*))
	(setq *aero* (instance AeroUpperRobot-robot :init))
      )
    (if (not (boundp '*aero-lower*))
	(setq *aero-lower* (instance AeroLowerRobot-robot :init))
      )
    (if (boundp '*aero-lower*)
	(send *aero* :newcoords
	      (send (send *aero-lower* :torso :end-coords)
		    :worldcoords))
      )
    (send (send *aero-lower* :links :WAIST_LINK0)
	  :assoc (send *aero* :links :WAIST_LINK0))
    (setq ((send *aero* :head :neck-p) . max-angle) 40)
    (setq ((send *aero-lower* :rarm :shoulder-y) . max-angle) 0)
    (setq ((send *aero-lower* :larm :shoulder-y) . min-angle) 0)
    )



  ;;;;;;;;;;;;;;;;;; for operating full body ;;;;;;;;;;;;;;;;;;
  (:angle-vector (&optional (angles nil))
    (if (eq angles nil)
	(return-from :angle-vector
		     (list (send *aero* :angle-vector) (send *aero-lower* :angle-vector)))
      )
    (send *aero* :angle-vector (elt angles 0))
    (send *aero-lower* :angle-vector (elt angles 1))
    angles)
  (:translate (p &optional (wrt :local))
    (send *aero-lower* :translate p wrt)
    )
  (:locate (p &optional (wrt :local))
    (send *aero-lower* :locate p wrt)
    )
  (:rotate (theta axis &optional (wrt :local))
    (send *aero-lower* :rotate theta axis wrt)
    )
  (:move-waist (&optional (waist-velocity (float-vector 0 0 0)) &key (draw-objects t))
    (let ((original-end-coords-list)
	  original-angle-vector
	  original-coords
	  )
      (when (< (norm waist-velocity) 3)
        (unless *real* (warn ";; :move-waist error: waist-velocity is too small~%") )
	(return-from :move-waist nil)
	)
      (setf original-angle-vector (send *aero-lower* :angle-vector))
      (setf original-coords (send *aero-lower* :copy-worldcoords))
      (dolist (link (list :rarm :larm :rleg :lleg))
        (push (send *aero-lower* link :end-coords :copy-worldcoords)
	      original-end-coords-list)
	)
      (setf original-end-coords-list (nreverse original-end-coords-list))
      (send *aero-lower* :translate waist-velocity)
      (dolist (link (list :rarm :larm :rleg :lleg))
        (unless (send *aero-lower* link :inverse-kinematics
		      (car original-end-coords-list) :rotation-axis nil :stop 150)
          (unless *real* (warn ";; move-waist IK failed >> ~S~%" link))
          (send *aero-lower* :angle-vector original-angle-vector)
          (send *aero-lower* :newcoords original-coords)
          (return-from :move-waist nil)
          )
        (setf original-end-coords-list (cdr original-end-coords-list))
        )
      (unless (not (send self :self-collision-check))
        (unless *real* (warn ";; move-waist failed at self collision check~%"))
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :newcoords original-coords)
        (return-from :move-waist nil)
        )
      (unless (eq (send self :cog-balance-check) t)
        (unless *real* (warn ";; move-waist failed: unbalacned after moving waist~%"))
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :newcoords original-coords)
        (return-from :move-waist nil)
        )
      (if (and draw-objects (boundp '*irtviewer*))
          (send *irtviewer* :draw-objects)
	)
      t))
  (:rotate-waist (angle axis &key (draw-objects t))
    (let ((original-end-coords-list)
	  original-angle-vector
	  original-coords
	  )
      (when (< (abs angle) (deg2rad 0.1))
        (unless *real* (warn ";; :rotate-waist error: angle is too small~%"))
	(return-from :rotate-waist nil)
	)
      (setf original-angle-vector (send *aero-lower* :angle-vector))
      (setf original-coords (send *aero-lower* :links :WAIST_LINK0 :copy-coords))
      (dolist (link (list :rarm :larm :rleg :lleg))
        (push (send *aero-lower* link :end-coords :copy-worldcoords)
              original-end-coords-list)
	)
      (setf original-end-coords-list (nreverse original-end-coords-list))
      (send *aero-lower* :links :WAIST_LINK0 :rotate angle axis)
      (dolist (link (list :rarm :larm :rleg :lleg))
        (unless (send *aero-lower* link :inverse-kinematics
		      (car original-end-coords-list) :rotation-axis nil)
          (unless *real* (warn ";; rotate-waist IK failed >> ~S~%" link))
          (send *aero-lower* :angle-vector original-angle-vector)
          (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
          (return-from :rotate-waist nil)
          )
        (setf original-end-coords-list (cdr original-end-coords-list))
        )
      (unless (not (send self :self-collision-check))
        (unless *real* (warn ";; rotate-waist failed at self collision check~%"))
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
        (return-from :rotate-waist nil)
        )
      (unless (eq (send self :cog-balance-check) t)
        (unless *real* (warn ";; rotate-waist failed: unbalacned after moving waist~%"))
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
        (return-from :move-waist nil)
        )
      (if (and draw-objects (boundp '*irtviewer*))
          (send *irtviewer* :draw-objects)
	)
      t))
  (:fix-leg-to-ground (&optional (legs '(:larm :rarm :lleg :rleg)))
    (let ((leg-height 0)
	  )
      (dolist (x legs)
        (setq leg-height (+ (aref (send (send self :end-ground-coords x) :pos) 2) leg-height))
	)
      (setq leg-height (/ leg-height (length legs)))
      (send self :translate (float-vector 0 0 (- leg-height)))
      ))



  ;;;;;;;;;;;;;;;;;; for upper manipulation ;;;;;;;;;;;;;;;;;;
  (:hand (&rest args)
    (if (null args) (return-from :hand nil))
    (let ((key (car args))
	  (nargs (cdr args))
	  )
      (unless (keywordp key) (return-from :hand nil))
      (case key
	    (:arms
	     (send *aero* :hand-poses :larm nargs)
	     (send *aero* :hand-poses :rarm nargs)
	     )
	    (:larm
	     (send *aero* :hand-poses :larm nargs))
	    (:rarm
	     (send *aero* :hand-poses :rarm nargs))
	    (t
	     nil)
	    )
      ))
  (:grasp (arm &optional (time 500))
    (send *aero* :grasp arm time)
    )
  (:ungrasp (arm &optional (time 500))
    (send *aero* :ungrasp arm time)
    )
  (:grasp-hard (arm &optional (time 1000) (x 0))
    (send *aero* :grasp-hard arm time x)
    )
  (:ungrasp-hard (arm &optional (time 1000) (x 0))
    (send *aero* :ungrasp-hard arm time x)
    )



  ;;;;;;;;;;;;;;;;;; for evaluating ;;;;;;;;;;;;;;;;;;
  (:self-collision-check ()
    (and (send *aero* :collision-check-all)
	 (send *aero-lower* :self-collision-check))
    )
  (:cog-balance-check (&optional (legs-on-grounds `(:larm :rarm :rleg :lleg)))
    (let (a b c p A1 B1 c-eval p-eval (tmp-legs)
	  )
      (setq tmp-legs legs-on-grounds)
      (setq p (send self :get-com))
      ;; check com
      (dolist (x legs-on-grounds)
	(setq c (send (send *aero-lower* (car tmp-legs) :end-coords :copy-worldcoords) :pos))
	(setq a (send (send *aero-lower* (cadr tmp-legs) :end-coords :copy-worldcoords) :pos))
	(setq b (send (send *aero-lower* (caddr tmp-legs) :end-coords :copy-worldcoords) :pos))
	(setq A1 (- (elt b 1) (elt a 1)))
	(setq B1 (- (elt b 0) (elt a 0)))
	(setq c-eval (if (> (- (* A1 (- (elt c 0) (elt a 0)))
			       (* B1 (- (elt c 1) (elt a 1)))) 0) t nil))
	(setq p-eval (if (> (- (* A1 (- (elt p 0) (elt a 0)))
			       (* B1 (- (elt p 1) (elt a 1)))) 0) t nil))
	(if (eq c-eval p-eval) nil (return-from :cog-balance-check p))
	(setq tmp-legs `(,@(cdr tmp-legs) ,(car tmp-legs)))
	)
      t))
  (:rate-balance (&optional (legs-on-grounds `(:rarm :rleg :lleg)))
    (let ((com-on-ground (send self :project-on-ground (send self :get-com)))
	  (get-3foot-center #f(0 0 0))
	  (tmp-legs legs-on-grounds)
	  edge-length
	  (edge-lengths `())
	  (s 0)
	  (area 1)
	  inner-radius
	  )
      (if (not (eq (send self :cog-balance-check legs-on-grounds) t))
	  (return-from :rate-balance 0.0)
	)
      (dolist (link legs-on-grounds)
	(setq foot-pos
	      (send self :project-on-ground
		    (send (send *aero-lower* (car tmp-legs)
				:end-coords :copy-worldcoords) :pos)))
	(setq edge-length
	      (norm (v- foot-pos
			(send self :project-on-ground
			      (send (send *aero-lower* (cadr tmp-legs)
					  :end-coords :copy-worldcoords) :pos)))))
	(setq get-3foot-center (v+ get-3foot-center foot-pos))
	(setq edge-lengths (push edge-length edge-lengths))
	(setq tmp-legs `(,@(cdr tmp-legs) ,(car tmp-legs)))
	)
      (dolist (x edge-lengths) (setq s (+ s x)) )
      (setq get-3foot-center (scale (/ 1.0 3) get-3foot-center))
      ;; (send (make-coords :pos get-3foot-center) :draw-on :flush t :size 500)
      (setq s (/ s 2))
      (dolist (x edge-lengths) (setq area (* (- s x) area)) )
      (setq area (sqrt (* s area)))
      (setq inner-radius (/ area s))
      (/ inner-radius (norm (v- get-3foot-center com-on-ground)))
      ))



  ;;;;;;;;;;;;;;;;;; for returning values ;;;;;;;;;;;;;;;;;;
  ;; calculate center of mass of the whole robot using
  ;; center of mass of aero upper and aero lower
  (:get-com ()
    (scale (/ 1 (+ (send *aero* :weight) (send *aero-lower* :weight)))
           (v+ (scale (send *aero* :weight) (send *aero* :centroid))
               (scale (send *aero-lower* :weight) (send *aero-lower* :centroid))))
    )
  ;; helper function for projecting a vector onto ground plane
  (:project-on-ground (in_coords)
    (float-vector (aref in_coords 0) (aref in_coords 1) 0)
    )
  ;; The end coords are at the wheels' axes
  ;; The end ground coords give the point that is touching the ground
  (:end-ground-coords (limb)
    (send (send (send *aero-lower* limb :end-coords) :copy-worldcoords)
	  :translate (float-vector 0 0 (- *wheel-radius*)) :world)
    ;; (eval `(send self :lower limb :end-coords))
    )
  (:error-vector ()
    (mapcar 'v- (send *ri* :actual-vector) (send *aero-fullbody* :angle-vector))
    )
  (:upper-error-sum (&optional (part :all))
    (setq sum 0)
    (setq upper-error-vector (elt (send self :error-vector) 0))
    (case part
          (:rarm
           (setq startIndex 0)
           (setq numJoints 11)
           )
          (:larm
           (setq startIndex 11)
           (setq numJoints 11)
           )
          (otherwise
           (setq startIndex 0)
           (setq numJoints (length upper-error-vector))
           )
          )
    (dotimes (i numjoints)
      (setq sum (+ sum (abs (aref upper-error-vector (+ i startIndex)))))
      )
    sum)



  ;;;;;;;;;;;;;;;;;; for viewing ;;;;;;;;;;;;;;;;;;
  (:draw-on ()
    ;; executing (objects *aero-fullbody*) is not going to work.
    ;; Use this function instead to visualize aero in irtviewer
    (objects *aero*)
    (objects *aero-lower*)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    )
  ;; draw center of mass projected onto ground plane
  (:draw-centroid-on-ground ()
    (let ((com-on-ground (make-coords
			  :pos (send self :project-on-ground (send self :get-com))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground))
  (:draw-upper-centroid-on-ground ()
    (let ((com-on-ground (make-coords
			  :pos (send self :project-on-ground (send *aero* :centroid))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground))
  (:draw-lower-centroid-on-ground ()
    (let ((com-on-ground (make-coords
			  :pos (send self :project-on-ground (send *aero-lower* :centroid))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground))
  (:print-lower-error-vector ()
    (setq lower-error-vector (elt error-vector 1))
    (setq lower-joint-names (send *aero-lower* :joint-list :name))
    (print "------- lower ------")
    (dotimes (i (length lower-error-vector))
      (format t "~s ~a~%" (elt lower-joint-names i) (aref lower-error-vector i))
      )
    )
  (:print-upper-error-vector (&optional (part :all))
    (setq upper-joint-names (send *aero* :joint-list :name))
    (setq upper-error-vector (elt (send self :error-vector) 0))
    (case part
          (:rarm
           (setq startIndex 0)
           (setq numJoints 11)
           )
          (:larm
           (setq startIndex 11)
           (setq numJoints 11)
           )
          (otherwise
           (setq startIndex 0)
           (setq numJoints (length upper-error-vector))
           )
          )
    (format t "------- upper ~a ------~%" part)
    (dotimes (i numjoints)
      (format t "~s ~a~%"
	      (elt upper-joint-names (+ i startIndex)) (aref upper-error-vector (+ i startIndex)))
      )
    (send self :print-upper-error-sum part)
    )
  (:print-upper-error-sum (&optional (part :all))
    (format t "~a sum ~a~%" part (send self :upper-error-sum part))
    )
  (:print-joint-error (upperOrLower jointID)
    (setq error-vector (send self :error-vector))
    (if (equal upperOrLower :upper)
        (setq error-vector (elt error-vector 0))
      (setq error-vector (elt error-vector 1))
      )
    (setq joint-names (send self upperOrLower :joint-list :name))
    (format t "~s ~a~%" (elt joint-names jointID) (aref error-vector jointID))
    )



  ;;;;;;;;;;;;;;;;;; poses ;;;;;;;;;;;;;;;;;;  
  (:reset-pose ()
    (send *aero* :reset-pose)
    (send *aero-lower* :reset-pose)
    (send *aero-lower* :links :WAIST_LINK0 :newcoords
          (make-coords :pos (send *aero-lower* :links :WAIST_LINK0 :pos) :rpy #f(0 0 0)))
    (send self :fix-leg-to-ground)
    )
  (:reset-manip-pose ()
    (send *aero* :reset-manip-pose)
    (send *aero-lower* :reset-manip-pose)
    (send *aero-lower* :links :WAIST_LINK0 :newcoords
          (make-coords :pos (send *aero-lower* :links :WAIST_LINK0 :pos) :rpy #f(0 0 0)))
    (send self :fix-leg-to-ground)
    )  

  ;; save the current pose of the robot
  (:save-pose ()
    (setq saved-pose (send self :angle-vector))
    )
  ;; load the previously saved pose of the robot
  (:load-pose ()
    (send self :angle-vector saved-pose)
    )
  ;; save the current pose and global position of the robot
  (:save-state ()
    (send self :save-pose)
    (setq saved-robot-coords (send *aero-lower* :copy-worldcoords))
    (setq saved-waist-coords (send *aero-lower* :links :WAIST_LINK0 :copy-coords))
    )
  ;; load the previously saved pose and global position of the robot
  (:load-state ()
    (send self :load-pose)
    (send *aero-lower* :newcoords (send saved-robot-coords :copy-coords))
    (send *aero-lower* :links :WAIST_LINK0 :newcoords saved-waist-coords)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    )
  (:match-ri ()
    (send self :angle-vector (send *ri* :actual-vector))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    )

  );; end of defmethod AeroRobot-robot



(defmethod aero-upper-interface
  (:angle-vector (av &optional (tm 5000))
    (if (eq (length av) 2)
	(setq av (concatenate float-vector (elt av 0) (elt av 1)))
      )
    (let ((upper-joint-names (list "r_shoulder_p_joint"
				   "r_shoulder_r_joint"
				   "r_shoulder_y_joint"
				   "r_elbow_joint"
				   "r_wrist_y_joint"
				   "r_wrist_p_joint"
				   "r_wrist_r_joint"
				   "r_thumb_joint"
				   "l_shoulder_p_joint"
				   "l_shoulder_r_joint"
				   "l_shoulder_y_joint"
				   "l_elbow_joint"
				   "l_wrist_y_joint"
				   "l_wrist_p_joint"
				   "l_wrist_r_joint"
				   "l_thumb_joint"
				   "waist_y_joint"
				   "waist_p_joint"
				   "waist_r_joint"
				   "neck_y_joint"
				   "neck_p_joint"
				   "neck_r_joint"))
	  (lower-joint-names (list "rr_hip_y_joint"
				   "rr_hip_p_joint"
				   "rr_knee_joint"
				   "fr_hip_y_joint"
				   "fr_hip_p_joint"
				   "fr_knee_joint"
				   "rl_hip_y_joint"
				   "rl_hip_p_joint"
				   "rl_knee_joint"
				   "fl_hip_y_joint"
				   "fl_hip_p_joint"
				   "fl_knee_joint"))
	  (send-av (float-vector
		    (deg2rad (elt av 0))
		    (deg2rad (elt av 1))
		    (deg2rad (elt av 2))
		    (deg2rad (elt av 3))
		    (deg2rad (elt av 4))
		    (deg2rad (elt av 5))
		    (deg2rad (elt av 6))
		    (deg2rad (elt av 7))
		    (deg2rad (elt av 11))
		    (deg2rad (elt av 12))
		    (deg2rad (elt av 13))
		    (deg2rad (elt av 14))
		    (deg2rad (elt av 15))
		    (deg2rad (elt av 16))
		    (deg2rad (elt av 17))
		    (deg2rad (elt av 18))
		    (deg2rad (elt av 22))
		    (deg2rad (elt av 23))
		    (deg2rad (elt av 24))
		    (deg2rad (elt av 25))
		    (deg2rad (elt av 26))
		    (deg2rad (elt av 27))))
	  (type-check (length av))
	  )
      (ros::publish "/aero_controller/command"
	  (case type-check
		(28 (instance trajectory_msgs::JointTrajectory
		     :init
		     :joint_names upper-joint-names
		     :points (list (instance trajectory_msgs::JointTrajectoryPoint
				      :init
				      :positions send-av
				      :time_from_start (ros::time (/ tm 1000.0)))
				   ))
		    )
		(13 (instance trajectory_msgs::JointTrajectory
		     :init
		     :joint_names lower-joint-names
		     :points (list (instance trajectory_msgs::JointTrajectoryPoint
				      :init
				      :positions (float-vector
						  (deg2rad (elt av 0))
						  (deg2rad (elt av 1))
						  (deg2rad (elt av 2))
						  (deg2rad (elt av 3))
						  (deg2rad (elt av 4))
						  (deg2rad (elt av 5))
						  (deg2rad (elt av 6))
						  (deg2rad (elt av 7))
						  (deg2rad (elt av 8))
						  (deg2rad (elt av 9))
						  (deg2rad (elt av 10))
						  (deg2rad (elt av 11)))
				      :time_from_start (ros::time (/ tm 1000.0)))
				   ))
		    )
		(41 (instance trajectory_msgs::JointTrajectory
		     :init
		     :joint_names (append upper-joint-names lower-joint-names)
		     :points (list (instance trajectory_msgs::JointTrajectoryPoint
				      :init
				      :positions (concatenate float-vector
						   send-av
						   (float-vector
						    (deg2rad (elt av 28))
						    (deg2rad (elt av 29))
						    (deg2rad (elt av 30))
						    (deg2rad (elt av 31))
						    (deg2rad (elt av 32))
						    (deg2rad (elt av 33))
						    (deg2rad (elt av 34))
						    (deg2rad (elt av 35))
						    (deg2rad (elt av 36))
						    (deg2rad (elt av 37))
						    (deg2rad (elt av 38))
						    (deg2rad (elt av 39))))
				      :time_from_start (ros::time (/ tm 1000.0)))
				   ))
		    )
		(t nil)
		)
	  )
     av))
  (:wheel-vector (av)
    (let ((wheel-joint-names (list "f_r_wheel_joint"
				   "r_r_wheel_joint"
				   "f_l_wheel_joint"
				   "r_l_wheel_joint"))
	  )
      (ros::publish "/aero_controller/wheel_command"
	  (instance trajectory_msgs::JointTrajectory
	     :init
	     :joint_names wheel-joint-names
	     :points (list (instance trajectory_msgs::JointTrajectoryPoint
				     :init
				     :positions av
				     :time_from_start (ros::time 0.0)))))
      av))
  (:wheel-on ()
    (ros::publish "/aero_controller/wheel_servo" (instance std_msgs::Bool :data t))
    )
  (:wheel-off ()
    (ros::publish "/aero_controller/wheel_servo" (instance std_msgs::Bool :data nil))
    )
  (:get-angle-vector (av)
    (list (float-vector
	   (rad2deg (cdr (assoc "r_shoulder_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "r_shoulder_r_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "r_shoulder_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "r_elbow_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "r_wrist_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "r_wrist_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "r_wrist_r_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "r_thumb_joint" av :test #'equal)) )
	   0
	   0
	   0
	   (rad2deg (cdr (assoc "l_shoulder_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "l_shoulder_r_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "l_shoulder_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "l_elbow_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "l_wrist_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "l_wrist_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "l_wrist_r_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "l_thumb_joint" av :test #'equal)) )
	   0
	   0
	   0
	   (rad2deg (cdr (assoc "waist_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "waist_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "waist_r_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "neck_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "neck_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "neck_r_joint" av :test #'equal)) )
	   )
	  (float-vector
	   (rad2deg (cdr (assoc "rr_hip_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "rr_hip_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "rr_knee_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "fr_hip_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "fr_hip_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "fr_knee_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "rl_hip_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "rl_hip_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "rl_knee_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "fl_hip_y_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "fl_hip_p_joint" av :test #'equal)) )
	   (rad2deg (cdr (assoc "fl_knee_joint" av :test #'equal)) )
	   0
	   )
	  ))
  (:go-pos (x y theta &optional (exec-large nil) (vel *wheel-velocity*)
	      (robot *aero-fullbody*) (sleep-step 200) (vel-step 60.0))
    (let (res
	  ui
	  )
      (if (eq *real* t) (send self :wheel-on))
      (unix:usleep (* 1000 300))
      (setq res (send self :gp-nil x y theta
		      exec-large (float vel) robot sleep-step vel-step))
      (unix:usleep (* 1000 300))
      (if (eq *real* t) (send self :wheel-off))
      res))
  (:gp-nil (&optional (x 0) (y 0) (theta 0)
		      (exec-large nil) (vel 308.0) (robot *aero-fullbody*)
		      (sleep-step 200) (vel-step 60.0))
    (let (time-xy
	  time-theta
	  ui
	  (~warn~ 1)
	  )
      (if (and (eq x 0) (eq y 0) (eq theta 0))
	  (return-from :gp-nil nil)
	)
      (setq time-xy (/ (* 100.0 2000 (sqrt (+ (* x x) (* y y))))
		       (* vel 120.0)))
      (if (not (= time-xy 0))
	  (progn
	    (cond ((or (> x 1000) (> y 1000))
		   (if (eq exec-large nil) (return-from :gp-nil -3) )
		   )
		  )
	    (let ((time-xy-left time-xy)
		  sleep-time
		  (current-vel 0.0)
		  )
	      (when *real*
		(do-until-key
		 (when (< time-xy-left 0.0) (return nil))
		 (setq current-vel (min (+ current-vel vel-step) vel))
		 (setq sleep-time (if (< time-xy-left sleep-step)
				      (round time-xy-left)
				    sleep-step))
		 (setq time-xy-left (- time-xy-left sleep-step))
		 (send self :wheel-vector
		       (scale (/ current-vel (abs (+ x y)))
			      (float-vector (- x y)
					    (- (+ x y))
					    (+ x y)
					    (- y x))))
		 (unix::usleep (* sleep-time 1000))
		 )
		(do-until-key
		 (when (< current-vel 1.0) (return nil))
		 (setq current-vel (max (- current-vel vel-step) 0.0))
		 (send self :wheel-vector
		       (scale (/ current-vel (abs (+ x y)))
			      (float-vector (- x y)
					    (- (+ x y))
					    (+ x y)
					    (- y x))))
		 (unix::usleep (* sleep-step 1000))
		 )
		(send self :wheel-vector #f(0 0 0 0))
		)
	      (return-from :gp-nil ~warn~))
	    ))
      (if (and (> (abs theta) 90) (eq exec-large nil))
	  (progn (setq ~warn~ -3) (setq theta 0))
	)
      (setq time-theta (abs (/ (* 100.0 2000 theta) (* vel 60.0))))
      (if (not (= time-theta 0))
	  (progn
	    (let ((time-theta-left time-theta)
		  sleep-time
		  (current-vel 0.0)
		  )
	      (when *real*
		(do-until-key
		 (when (< time-theta-left 0.0) (return nil))
		 (setq current-vel (min (+ current-vel vel-step) vel))
		 (setq sleep-time (if (< time-theta-left sleep-step)
				      (round time-theta-left)
				    sleep-step))
		 (setq time-theta-left (- time-theta-left sleep-step))
		 (send self :wheel-vector
		       (scale (/ theta (abs theta))
			      (float-vector current-vel
					    (- current-vel)
					    (- current-vel)
					    current-vel)))
		 (unix::usleep (* sleep-time 1000))
		 (setq time-theta-left (- time-theta-left sleep-step))
		 )
		(do-until-key
		 (when (< current-vel 1.0) (return nil))
		 (setq current-vel (max (- current-vel vel-step) 0.0))
		 (send self :wheel-vector
		       (scale (/ theta (abs theta))
			      (float-vector current-vel
					    (- current-vel)
					    (- current-vel)
					    current-vel)))
		 (unix::usleep (* sleep-step 1000))
		 )
		(send self :wheel-vector #f(0 0 0 0)))
	      (return-from :gp-nil ~warn~))
	    ))
      ~warn~))

  );; end of defmethod aero-upper-interface



(defun aero-init (&rest args)
  (when (not (boundp '*ri*))
    (let ((disp (copy-object x::*display*))
	  )
      (setf x::*display* 0)
      (setq *ri* (instance* aero-upper-interface :init args))
      (setf x::*display* disp)
      ))
  (if (not (boundp '*aero-fullbody*))
      (setq *aero-fullbody* (instance AeroRobot-robot :init))
    )
  (send *aero-fullbody* :angle-vector (send *ri* :actual-vector))
  )



(defvar *wheel-velocity* 308.0)
(defvar *wheel-radius* 44.63875)
