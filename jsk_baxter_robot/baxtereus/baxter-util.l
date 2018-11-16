(require :baxter "package://baxtereus/baxter.l")

(defmethod baxter-robot
  (:ik-prepared-poses
    (&optional (poses :null))
    "(:ik-prepared-poses) returns (get self :ik-prepared-poses), unless it is :null or (:untuck-pose)
    (:ik-prepared-poses nil) set :null to (get self :ik-prepared-poses)
    (:ik-prepared-poses :any-posture) set :any-posutre to (get self :ik-prepared-poses)
    "
    (let ((ik-prepared-poses (or (get self :ik-prepared-poses) '(:untuck-pose))))
      (ros::ros-debug "[:ik-prepared-poses] called with ~A" poses)
      (cond ((eq poses :null)
             (ros::ros-debug "[:ik-prepared-poses] returns ~A" (if (eq ik-prepared-poses :null) nil ik-prepared-poses))
             (return-from :ik-prepared-poses (if (eq ik-prepared-poses :null) nil ik-prepared-poses)))
            ((eq poses nil)
             (setf (get self :ik-prepared-poses) :null)
             (ros::ros-debug "[:ik-prepared-poses] returns nil")
             (return-from :ik-prepared-poses nil))
            (t
              (setf (get self :ik-prepared-poses) poses)
              (ros::ros-debug "[:ik-prepared-poses] returns ~A" poses)
              (return-from :ik-prepared-poses poses))
            )))
  (:inverse-kinematics
   (target-coords &rest args &key (avoid-collision-distance 5) (warnp) (dump-command) &allow-other-keys)
   "Compute Inverse Kinematics with some strategies.
    The computation runs until the IK is solved as below:

      1. From current pose.
      2. Split points between current and target coords from each.
      3. From prepared poses. Poses in `:ik-prepared-poses` methods are used,
          and if it is not defined `:untuck-pose` is used."
   (ros::ros-debug "call ik of ~A" baxter-robot)
   (let ((r) (prev-av (send self :angle-vector)))
     (setq r (send-super* :inverse-kinematics target-coords :avoid-collision-distance avoid-collision-distance :warnp warnp :dump-command dump-command args))
     (unless r ;;
       (format *error-output* "; failed for normal ik, starting from relaxed position~%")
       (let  ((current-coords (send (cadr (memq :move-target args)) :copy-worldcoords)))
         (send-super* :inverse-kinematics current-coords :rotation-axis nil :avoid-nspace-gain 0.1 :avoid-weight-gain 0.1 :stop 200 :avoid-collision-distance avoid-collision-distance args)
         (setq r (send-super* :inverse-kinematics target-coords :warnp warnp :dump-command dump-command args))
         (if (and (null r) (or (null (memq :revert-if-fail args)) (cadr (memq :revert-if-fail args))))
             ;; when fail and :rever-if-fail is nil
             (send self :angle-vector prev-av))
         ))
     (unless r ;;
       (format *error-output* "; failed for normal ik, try to move arms very slowly~%")
       (let  ((step 0.0)
              (current-coords (send (cadr (memq :move-target args)) :copy-worldcoords)))
         (setq r t)
         (while (and r (<= step 1.0))
           (setq r (send-super* :inverse-kinematics (midcoords step current-coords target-coords) :avoid-collision-distance avoid-collision-distance :warnp warnp :dump-command dump-command args))
           (incf step 0.01))
         (unless r (send self :angle-vector prev-av))))
     (unless r ;; start from ik-frendly position
       (format *error-output* "; failed for slow ik, try to start from prepared poses~%")
       (let* ((move-joints (send-all (cadr (memq :link-list args)) :joint))
              (av (mapcar #'(lambda (j) (send j :joint-angle)) (send self :joint-list)))
              (ik-prepared-poses (send self :ik-prepared-poses)))
         (dolist (pose ik-prepared-poses)
           (unless r
             (format *error-output* "; starting from prepared pose '~A'~%" pose)
             (send self pose)
             (mapcar #'(lambda (j a)
                         (if (not (memq j move-joints))
                           (send j :joint-angle a)))
                     (send self :joint-list) av)
             (setq r (send-super* :inverse-kinematics target-coords :avoid-collision-distance avoid-collision-distance :warnp warnp :dump-command dump-command args))
             )
           )
         (when (and (null r) (or (null (memq :revert-if-fail args)) (cadr (memq :revert-if-fail args))))
             (when dump-command
               (format *error-output* "; failed all ik attempts, dump all data to /tmp/irtmodel-ik-~A~%" (unix::getpid))
               (format *error-output* ";      base coords ~A~%" (send self :worldcoords))
               (format *error-output* ";    target coords ~A~%" target-coords))
             ;; when fail and :rever-if-fail is nil
             (send self :angle-vector prev-av))
         ))
     r))
  (:l/r-reverse
    (av)
    "return the other arm's angle vector"
    (let ((av-tmp (copy-list av)) index)
      (dotimes (i 4)
        (setq index (* i 2))
        (setf (elt av-tmp index) (- (elt av-tmp index))))
      av-tmp))
  )

#|
(baxter)
(objects (list *baxter*))
(setq av0 #f(0.0 97.3149 -121.216 -143.873 138.766 -5.67992 -73.1565 -16.2255 -20.0 -25.0 40.0 60.0 20.0 80.0 0.0))
(send *baxter* :angle-vector av0)
(send *viewer* :draw-objects)
(send *baxter* :larm :move-end-pos #f(160 0 0) :world :debug-view nil) ;; ng
(send *viewer* :draw-objects)
|#

(defclass baxter-robot-safe
  :super baxter-robot
  :slots ())
(defmethod baxter-robot-safe
  (:init (&rest args)
         (send-super* :init args)
         ;; add screen and remove base from all links
         (setq collision-avoidance-links (remove-if #'(lambda (x) (string= (send x :name) "base")) (append (list (send self :screen_lk)) (send self :links))))
         (setq larm-collision-avoidance-links collision-avoidance-links)
         (setq rarm-collision-avoidance-links collision-avoidance-links)
	 (send (send self :left_w2) :max-joint-velocity 1)
	 (send (send self :left_w1) :max-joint-velocity 2)
	 (send (send self :left_w0) :max-joint-velocity 2.5)
	 (send (send self :left_e0) :max-joint-velocity 1)
	 (send (send self :left_s0) :max-joint-velocity 1.4)

	 (send (send self :right_w2) :max-joint-velocity 1)
	 (send (send self :right_w1) :max-joint-velocity 2)
	 (send (send self :right_w0) :max-joint-velocity 2.5)
	 (send (send self :right_e0) :max-joint-velocity 1)
	 (send (send self :right_s0) :max-joint-velocity 1.4)
         )
  ;; collision check methods
  (:collision-check-pairs
   (&rest args)
   (let (pairs)
     (if args (warning-message 1 ":collision-check-pairs is re-defined in baxter-robot-safe class and not intend to take any arguments ~A" args))
     (setq pairs (send-super :collision-check-pairs :links collision-avoidance-links))
     ;; remove collision between "upper_elbow" and "upper_forearm"
     (setq pairs (remove-if #'(lambda (x)
                                (subsetp (list "upper_elbow" "upper_forearm")
                                         (list (send (car x) :name) (send (cdr x) :name))
                                         :test #'substringp))
                            pairs))
     pairs))
  )

