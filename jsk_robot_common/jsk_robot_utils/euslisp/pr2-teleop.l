(ros::load-ros-manifest "sensor_msgs")

(defvar *max-pos* 3.0)
(defvar *max-rot* 0.015)
(defvar *send-time* 1000)

(defun teleop-cb (msg)
  ;; mode callback
  (let* ((axes (send msg :axes))
         (Lstick (coerce (subseq axes 0 2) float-vector))
         (Rstick (coerce (subseq axes 2 4) float-vector))
         (buttons (send msg :buttons))

         (select (elt buttons 0))
         (start (elt buttons 3))
         (L2 (elt buttons 8))
         (R2 (elt buttons 9))
         (R1 (elt buttons 11))
         (right/left       (elt Rstick 0))
         (forward/backward (elt Rstick 1))
         (up/down          (elt Lstick 1))

         (circle   (elt buttons 13)) ;; roll+
         (square   (elt buttons 15)) ;; roll-
         (triangle (elt buttons 12)) ;; pitch-
         (cross    (elt buttons 14)) ;; pitch+
         (right    (elt buttons  7)) ;; yaw+
         (left     (elt buttons  5)) ;; yaw-

         (roll  (* *max-rot* (- circle square)))
         (pitch (* *max-rot* (- cross triangle)))
         (yaw   (* *max-rot* (- right left)))

         (up       (elt buttons  4)) ;; grasp
         (down     (elt buttons  6)) ;; release

         arm pos
         target-coords)

    ;; toggle move-arm
    (when (and (= start 1) (not (= start *prev-start*)))
      (setf (elt *mode* 0) (not (elt *mode* 0)))
      (when *debug* (warning-message 1 "start button pressed! ~A" *mode*)))
    (when (and (= select 1) (not (= select *prev-select*)))
      (setf (elt *mode* 1) (not (elt *mode* 1)))
      (when *debug* (warning-message 1 "select button pressed! ~A" *mode*)))
    (setq *prev-start* start
          *prev-select* select)

    (setq pos (scale *max-pos* (float-vector forward/backward
                                             right/left
                                             up/down)))
    (when (and (eq R1 0)
               (eq R2 0)
               (eq L2 0))
      (when (elt *mode* 0) ;; right arm on
        (setq arm :rarm)
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
        (send *robot* arm :inverse-kinematics target-coords
              :revert-if-fail t
              :stop 100
              :debug-view nil))
      (when (elt *mode* 1) ;; left arm on
        (setq arm :larm)
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
        (send *robot* arm :inverse-kinematics target-coords
              :revert-if-fail t
              :stop 100
              :debug-view nil))
      )

    (send *ri* :angle-vector (send *robot* :angle-vector) *send-time*)
    (if *debug* (send *irtviewer* :draw-objects))
    ))

(defun grasp-loop ()
  (when (or (and (not *prev-r-grasp*) *r-grasp*)
            (and *prev-r-grasp* (not *r-grasp*))) ;; when state changed
    (if *r-grasp*
        (send *ri* :start-grasp :rarm :gain-percentage 40 :target-error 0.01)
        (progn
          (send *ri* :stop-grasp :rarm)
          (send *robot* :release :rarm)
          (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
          (send *ri* :wait-interpolation)
          )))
  (when (or (and (not *prev-l-grasp*) *l-grasp*)
            (and *prev-l-grasp* (not *l-grasp*))) ;; when state changed
    (if *l-grasp*
        (send *ri* :start-grasp :larm :gain-percentage 40 :target-error 0.01)
        (progn
          (send *ri* :stop-grasp :larm)
          (send *robot* :release :larm)
          (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
          (send *ri* :wait-interpolation)
          )))
  (setq *prev-r-grasp* *r-grasp*
        *prev-l-grasp* *l-grasp*)
  t)

(defun reset-loop (&key (reset-type :reset-manip-pose))
  (when *reset*
    (send *ri* :angle-vector (send *robot* reset-type) 5000)
    (send *ri* :wait-interpolation))
  (setq *reset* nil)
  t)

(defun look-at-loop ()
  (when (or *look-right-hand*
            *look-left-hand*)
    (let* ((arm (cond (*look-right-hand* :rarm)
                      (*look-left-hand*  :larm))))
      (send *robot* :head :look-at (send *robot* arm :end-coords :worldpos))
      (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation))

    (setq *look-right-hand* nil
          *look-left-hand*  nil)
    )
  )

(defun test-teleop ()
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :wait-interpolation)
  (setq *prev-r-grasp* nil
        *prev-l-grasp* nil
        *r-grasp*      nil
        *l-grasp*      nil
        *look-right-hand* nil
        *look-left-hand*  nil
        *reset* nil
        *prev-start* 0
        *prev-select* 0)
  (setq *mode* (list nil nil))
  (do-until-key
      (ros::spin-once)
    ;; (grasp-loop)
    ;; (reset-loop)
    ;; (look-at-loop)
    (ros::sleep))
  )

(defun pr2-teleop-setup ()
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  (setq *robot* *pr2*)
  (when *debug*
    (unless (boundp '*irtviewer*)
      (make-irtviewer)
      (objects (list *robot*)))
    )
  (ros::rate 50)
  (ros::subscribe "/joy" sensor_msgs::Joy #'teleop-cb)
  )

(pr2-teleop-setup)
