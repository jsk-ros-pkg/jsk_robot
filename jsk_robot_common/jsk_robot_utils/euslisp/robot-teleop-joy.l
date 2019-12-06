(ros::load-ros-manifest "sensor_msgs")

(defvar *max-pos* 3.0)
(defvar *max-rot* 0.015)
(defvar *send-time* 1000)
(defvar *head-controller-gain* 0.30)
(defvar *debug-teleop-joy* nil)

(defun teleop-joy-cb (msg)
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

         (right    (elt buttons  5)) ;; roll+
         (left     (elt buttons  7)) ;; roll-
         (triangle (elt buttons 12)) ;; pitch+
         (cross    (elt buttons 14)) ;; pitch-
         (square   (elt buttons 15)) ;; yaw+
         (circle   (elt buttons 13)) ;; yaw-

         (roll  (* *max-rot* (- right left)))
         (pitch (* *max-rot* (- triangle cross)))
         (yaw   (* *max-rot* (- square circle)))

         (up       (elt buttons  4)) ;; grasp
         (down     (elt buttons  6)) ;; release

         arm pos
         target-coords)

    ;; reset pose
    (when (and *reset-loop-enable* (eq L2 1) (eq R2 1) (eq left 1))
      (setq *reset* t)
      (return-from teleop-joy-cb nil))

    ;; look at hand
    (when (and *look-at-loop-enable* (= R1 1) (= up 1))
      (setq *look-hand* t)
      (return-from teleop-joy-cb nil))

    ;; move head
    (when (and *move-head-enable* (= R1 1))
      (let* ((neck-p (send *robot* :head-neck-p :joint-angle))
             (neck-y (send *robot* :head-neck-y :joint-angle)))
        (send *robot* :head-neck-p :joint-angle
              (min *head-neck-p-max-angle*
                   (max *head-neck-p-min-angle*
                        (- neck-p (* *head-controller-gain* up/down)))))
        (send *robot* :head-neck-y :joint-angle
              (min *head-neck-y-max-angle*
                   (max *head-neck-y-min-angle*
                        (+ neck-y (* *head-controller-gain* right/left)))))
        ))

    ;; grasp/release hand
    (when (and *grasp-loop-enable*)
      (cond
        ((= circle 1)
         (when (= R2 1) (setq *r-grasp* nil))
         (when (= L2 1) (setq *l-grasp* nil)))
        ((= square 1)
         (when (= R2 1) (setq *r-grasp* t))
         (when (= L2 1) (setq *l-grasp* t)))))

    ;; move arm
    ;; toggle move-arm
    (when (and (= start 1) (not (= start *prev-start*)))
      (setf (elt *mode* 0) (not (elt *mode* 0))) ;; right arm on/off
      (when *debug-teleop-joy* (warning-message 1 "start button pressed! ~A" *mode*)))
    (when (and (= select 1) (not (= select *prev-select*)))
      (setf (elt *mode* 1) (not (elt *mode* 1))) ;; left arm on/off
      (when *debug-teleop-joy* (warning-message 1 "select button pressed! ~A" *mode*)))
    (setq *prev-start* start
          *prev-select* select)
    (setq pos (scale *max-pos* (float-vector forward/backward
                                             right/left
                                             up/down)))
    (when (and (= R1 0)
               (= R2 0)
               (= L2 0))
      (mapc #'(lambda (mode arm)
                (when mode ;; right/left arm on
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
                        :debug-view nil)))
            *mode* '(:rarm :larm))
      )

    (send *ri* :angle-vector (send *robot* :angle-vector) *send-time*)
    (if *debug-teleop-joy* (send *irtviewer* :draw-objects))
    ))

(defun grasp-loop ()
  (mapc #'(lambda (l)
            (let ((arm        (car l))
                  (prev-grasp (cadr l))
                  (grasp      (caddr l)))
              (when (or (and (not prev-grasp) grasp)
                        (and prev-grasp (not grasp))) ;; when state changed
                (if grasp
                    (progn
                      (if *debug-teleop-joy* (warning-message 1 "grasp ~A~%" arm))
                      (send *ri* :start-grasp arm :gain-percentage 40 :target-error 0.01))
                  (progn
                    (if *debug-teleop-joy* (warning-message 1 "release ~A~%" arm))
                    (send *ri* :stop-grasp arm)))
                )))
        (list (list :rarm *prev-r-grasp* *r-grasp*)
              (list :larm *prev-l-grasp* *l-grasp*)))
  (setq *prev-r-grasp* *r-grasp*
        *prev-l-grasp* *l-grasp*)
  t)

(defun reset-loop (&key (reset-pose :reset-manip-pose))
  (when *reset*
    (if *debug-teleop-joy* (warning-message 1 "execute ~A~%" reset-pose))
    (send *ri* :angle-vector (send *robot* reset-pose) 5000)
    (send *ri* :wait-interpolation))
  (setq *reset* nil)
  t)

(defun look-at-loop ()
  (when *look-hand*
    (let* ((arm (cond ((eq *which-hand* :larm) :rarm)
                      ((eq *which-hand* :rarm) :larm)
                      (t :larm))))
      (if *debug-teleop-joy* (warning-message 1 "look at ~A~%" arm))
      (send *robot* :head :look-at (send *robot* arm :end-coords :worldpos))
      (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
      (send *ri* :wait-interpolation)
      (setq *which-hand* arm))
    (setq *look-hand* nil)
    )
  )

(defun run-teleop-joy (&key (reset? t) (reset-pose :reset-manip-pose))
  (when reset?
    (send *ri* :angle-vector (send *robot* reset-pose) 5000)
    (send *ri* :wait-interpolation))
  (setq *prev-r-grasp* nil
        *prev-l-grasp* nil
        *r-grasp*      nil
        *l-grasp*      nil
        *look-hand*  nil
        *which-hand* nil
        *reset* nil
        *prev-start* 0
        *prev-select* 0
        *mode* (list nil nil))
  (if *debug-teleop-joy* (warning-message 1 "start teleop-joy~%"))
  (do-until-key
    (ros::spin-once)
    (if *grasp-loop-enable*   (grasp-loop))
    (if *reset-loop-enable*   (reset-loop))
    (if *look-at-loop-enable* (look-at-loop))
    (ros::sleep))
  )

(defun teleop-joy-setup ()
  ;; options
  (setq *grasp-loop-enable* nil
        *reset-loop-enable* nil
        *look-at-loop-enable* nil
        *move-head-enable* nil)
  ;; head min/max angle
  (setq *head-neck-p-max-angle* (- (send *robot* :head-neck-p :max-angle) 1.0)
        *head-neck-p-min-angle* (+ (send *robot* :head-neck-p :min-angle) 1.0)
        *head-neck-y-max-angle* (- (send *robot* :head-neck-y :max-angle) 1.0)
        *head-neck-y-min-angle* (+ (send *robot* :head-neck-y :min-angle) 1.0))
  ;; spin rate
  (ros::rate 50)
  ;; subscribe joy topic
  (ros::subscribe "/joy" sensor_msgs::Joy #'teleop-joy-cb)
  )
