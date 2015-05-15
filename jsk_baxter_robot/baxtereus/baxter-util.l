(require :baxter "package://baxtereus/baxter.l")

(defmethod baxter-robot
  (:inverse-kinematics
   (target-coords &rest args &key (avoid-collision-distance 5) &allow-other-keys)
   (let ((r))
     (setq r (send-super* :inverse-kinematics target-coords :avoid-collision-distance avoid-collision-distance :warnp nil :dump-command nil args))
     (unless r ;;
       (format *error-output* "; failed for normal ik, staring from relaxed position~%")
       (let  ((step 0.0)
              (current-coords (send (cadr (memq :move-target args)) :copy-worldcoords))
              (av (send self :angle-vector)))
         (send-super* :inverse-kinematics current-coords :rotation-axis nil :avoid-nspace-gain 0.1 :avoid-weight-gain 0.1 :stop 200 :avoid-collision-distance avoid-collision-distance :debug-view nil args)
         (setq r (send-super* :inverse-kinematics target-coords :warnp nil :dump-command nil args))
         (if (and (null r) (and (memq :revert-if-fail args)) (null (cadr (memq :revert-if-fail args))))
             ;; when fail and :rever-if-fail is nil
             (send robot :angle-vector av))
         ))
     (unless r ;;
       (format *error-output* "; failed for normal ik, try to move arms vely slowly~%")
       (let  ((step 0.0)
              (current-coords (send (cadr (memq :move-target args)) :copy-worldcoords))
              (av (send self :angle-vector)))
         (setq r t)
         (while (and r (<= step 1.0))
           (setq r (send-super* :inverse-kinematics (midcoords step current-coords target-coords) :avoid-collision-distance avoid-collision-distance :debug-view nil :warnp nil :dump-command nil args))
           (incf step 0.01))
         (unless r (send self :angle-vector av))))
     (unless r ;; start from ik-frendly position
       (format *error-output* "; failed for slow ik, try to start from good position~%")
       (let* ((move-joints (send-all (cadr (memq :link-list args)) :joint))
              (av (mapcar #'(lambda (j) (send j :joint-angle)) (send self :joint-list))))
         (send self :untuck-pose)
         (mapcar #'(lambda (j a)
                     (if (not (memq j move-joints))
                         (send j :joint-angle a)))
                 (send self :joint-list) av)
         (setq r (send-super* :inverse-kinematics target-coords :avoid-collision-distance avoid-collision-distance :warnp nil :dump-command nil args))
         (if (and (null r) (and (memq :revert-if-fail args)) (null (cadr (memq :revert-if-fail args))))
             ;; when fail and :rever-if-fail is nil
             (send robot :angle-vector av))
         ))
     r)))

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
         (setq collision-avoidance-links (list (send self :screen_lk)
                                               (send self :larm :wrist-r :child-link)
                                               (send self :larm :elbow-p :child-link)
                                               (send self :larm :elbow-r :child-link)
                                               (send self :rarm :wrist-r :child-link)
                                               (send self :rarm :elbow-p :child-link)
                                               (send self :rarm :elbow-r :child-link)
					       ))
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
  (:self-collision-check (&key (pairs (send self :collision-check-pairs :links collision-avoidance-links)) &rest args &allow-other-keys)
                         (send-super* :self-collision-check :pairs pairs args))
  )

