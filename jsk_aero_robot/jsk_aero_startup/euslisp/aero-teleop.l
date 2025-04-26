#!/usr/bin/env roseus
;; (ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "sensor_msgs")

(defvar *max-pos* 3.0)
(defvar *max-rot* 0.015)
(defvar *send-time* 1000)
(defvar *head-controller-gain* 0.30)
(defvar *ik-type* :arm-ik)
(defvar *debug* t)

(defun teleop-cb (msg &key (ik-type *ik-type*))
  ;; mode callback
  (let* ((axes (send msg :axes))
	 (Lstick (coerce (subseq axes 0 2) float-vector))
	 (Rstick (float-vector (elt axes 2) (elt axes 5)))
	 (buttons (send msg :buttons))

	 (select (elt buttons 8))
	 (start (elt buttons 9))
	 (L2 (elt buttons 6))
	 (R2 (elt buttons 7))
	 (R1 (elt buttons 5))
	 (L1 (elt buttons 4))
	 (right/left       (elt Rstick 0))
	 (up/down          (elt Rstick 1))
	 (forward/backward (elt Lstick 1))

	 (circle   (elt buttons 2)) ;; roll+
	 (square   (elt buttons 0)) ;; roll-
	 (triangle (elt buttons 3)) ;; pitch-
	 (cross    (elt buttons 1)) ;; pitch+
	 (right    (- (elt axes 9))) ;; yaw+
	 (left     (elt axes 9)) ;; yaw-

	 (roll  (* *max-rot* (- circle square)))
	 (pitch (* *max-rot* (- cross triangle)))
	 (yaw   (* *max-rot* (- right left)))

	 (up       (elt axes 10)) ;; grasp
	 (down     (- (elt axes 10))) ;; release
	 (switch   (elt buttons 10)) ;; lstick button
	 arm pos
	 target-coords)

    (when (= switch 1)
      (send *robot* :angle-vector (send *ri* :state :potentio-vector)))

    (when (= start 1)
      (setq *reset* t)
      (return-from teleop-cb nil))

    (when (or (= R2 1)
	      (= L2 1)
	      (= R1 1))
      (cond
       ;; ((= R1 1)
       ;;  (let* ((neck-p (send *robot* :neck_p_joint :joint-angle))
       ;;         (neck-y (send *robot* :neck_y_joint :joint-angle)))
       ;;    (send *robot* :neck_p_joint :joint-angle
       ;;  	(min 54 (max -14 (- neck-p (* *head-controller-gain* up/down)))))
       ;;    (send *robot* :neck_y_joint :joint-angle
       ;;  	(min 118 (max -118 (+ neck-y (* *head-controller-gain* right/left)))))
       ;;    ))
       (t
	(when (= select 1)
	  (cond
	   ((= R2 1)
	    (setq *look-right-hand* t))
	   ((= L2 1)
	    (setq *look-left-hand* t)))
	  (return-from teleop-cb nil)
	  )

	(setq pos (scale *max-pos* (float-vector forward/backward
						 right/left
						 up/down)))
	(cond
	 ((= R2 1)
	  (setq arm :rarm)
	  (cond ((= up 1) (setq *r-grasp* t))
		((= down 1) (setq *r-grasp* nil))))
	 ((= L2 1)
	  (setq arm :larm)
	  (cond ((= up 1) (setq *l-grasp* t))
		((= down 1) (setq *l-grasp* nil))))
	 ((= R1 1)
          (return-from teleop-cb nil)))

	;; (send *robot* arm :move-end-pos pos :world)
	;; (send *robot* arm :move-end-rot rot axis :world)
	(setq target-coords
	      (send
	       (send
		(send
		 (send
		  (send *robot* arm :end-coords :copy-worldcoords)
		  :translate pos :world)
		 :rotate roll :x :world)
		:rotate pitch :y :world)
	       :rotate yaw :z :world))

        (if (= L1 1)
            (send *robot* :inverse-kinematics
                  (if (eq arm :rarm)
                      (list
                       target-coords
                       (send (send *robot* :larm :end-coords) :copy-worldcoords))
                    (list
                     (send (send *robot* :rarm :end-coords) :copy-worldcoords)
                     target-coords))
                  :ik-group :both-arm-whole-body
                  :revert-if-fail nil
                  :stop 100
                  :debug-view nil)
          (send *robot* :inverse-kinematics target-coords
                :ik-group (if (eq arm :rarm) :rarm-whole-body :larm-whole-body)
                :revert-if-fail t
                :stop 100
                :debug-view nil)
          )
        (send *robot* :look-at-hand arm)
        )
       ))

    (send *ri* :angle-vector (send *robot* :angle-vector) *send-time*
          :default-controller (ros::time 0))
    (if *debug* (send *irtviewer* :draw-objects))
    ))

(defun grasp-loop ()
  (when (or (and (not *prev-r-grasp*) *r-grasp*)
	    (and *prev-r-grasp* (not *r-grasp*))) ;; when state changed
    (if *r-grasp*
	(send *ri* :start-grasp :rarm)
      (progn
	(send *ri* :stop-grasp :rarm)
	(send *ri* :angle-vector (send *robot* :angle-vector) 2000
              :default-controller (ros::time 0))
	(send *ri* :wait-interpolation)
	)))
  (when (or (and (not *prev-l-grasp*) *l-grasp*)
	    (and *prev-l-grasp* (not *l-grasp*))) ;; when state changed
    (if *l-grasp*
	(send *ri* :start-grasp :larm)
      (progn
	(send *ri* :stop-grasp :larm)
	;; (send *robot* :stop-grasp :larm)
	(send *ri* :angle-vector (send *robot* :angle-vector) 2000
              :default-controller (ros::time 0))
	(send *ri* :wait-interpolation)
	)))
  (setq *prev-r-grasp* *r-grasp*
	*prev-l-grasp* *l-grasp*)
  t)

(defun reset-loop (&key (reset-type :reset-pose))
  (when *reset*
    (send *ri* :angle-vector (send *robot* reset-type) 5000
          :default-controller (ros::time 0))
    (send *ri* :wait-interpolation))
  (setq *reset* nil)
  t)

(defun look-at-loop ()
  (when (or *look-right-hand*
	    *look-left-hand*)
    (let* ((arm (cond (*look-right-hand* :rarm)
		      (*look-left-hand*  :larm))))
      (send *robot* :head :look-at (send *robot* arm :end-coords :worldpos))
      (send *ri* :angle-vector (send *robot* :angle-vector) 2000
            :default-controller (ros::time 0))
      (send *ri* :wait-interpolation))

    (setq *look-right-hand* nil
	  *look-left-hand*  nil)
    )
  )

(defun test-teleop ()
  (format t "~A~%" "start aero-teleop-ik")
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000
        :default-controller (ros::time 0))
  (send *ri* :wait-interpolation)
  (setq *prev-r-grasp* nil
	*prev-l-grasp* nil
	*r-grasp*      nil
	*l-grasp*      nil
	*look-right-hand* nil
	*look-left-hand*  nil
	*reset* nil)
  (do-until-key
   (ros::spin-once)   
   (grasp-loop)
   (reset-loop)
   (look-at-loop)
   (ros::sleep))
  )

(defun aero-teleop-setup ()
  (load "package://aeroeus/aero-interface.l")
  (aero-init)
  (setq *robot* *aero*)
  ;; (objects (list *robot*))
  (ros::rate 0.3)
  (ros::subscribe "/joy" sensor_msgs::Joy #'teleop-cb)
  )

;; (ros::roseus "aero_teleop_ik")
(aero-teleop-setup)
(test-teleop)
