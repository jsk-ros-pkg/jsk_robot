(require :go1 "package://unitreeeus/go1.l")

(defun go1 () (setq *go1* (instance go1_description-robot :init)))

;; euscollada can not generate end-coords without limbs...
;; head-end-coords:
;;  parent : camera_optical_face_lk
;;   translate : [0, -12.5, -6.7]
;;   rotate : [0, 1, 0, 0]
(defmethod go1_description-robot
  (:head-end-coords (&rest args) (forward-message-to camera_optical_face_lk args))
  )

;; define ik methods
(defmethod go1_description-robot
  (:fullbody-inverse-kinematics
   ;; see https://github.com/euslisp/jskeus/pull/349 for detail
   (target-coords move-target &rest args &key
                  (rotation-axis t)
                  (translation-axis t)
                  (stop 15)
                  (thre 3)
                  (rthre 3)
                  &allow-other-keys)
   (send-super* :fullbody-inverse-kinematics
                (list target-coords
                      (send self :larm :end-coords :copy-worldcoords)
                      (send self :rarm :end-coords :copy-worldcoords)
                      (send self :lleg :end-coords :copy-worldcoords)
                      (send self :rleg :end-coords :copy-worldcoords))
                :move-target
                (list move-target
                      (send self :larm :end-coords)
                      (send self :rarm :end-coords)
                      (send self :lleg :end-coords)
                      (send self :rleg :end-coords))
                :link-list
                (list (send self :link-list move-target)
                      (send self :link-list (send self :larm :end-coords :parent))
                      (send self :link-list (send self :rarm :end-coords :parent))
                      (send self :link-list (send self :lleg :end-coords :parent))
                      (send self :link-list (send self :rleg :end-coords :parent)))
                :rotation-axis (list rotation-axis nil nil nil nil)
                :translation-axis (list translation-axis t t t t )
                :dump-command nil
                :stop stop
                :thre (make-list 5 :initial-element thre)
                :rthre (make-list 5 :initial-element (deg2rad rthre))
                :cog-gain 0.0 ;; disable CoG control
                :centroid-thre *inf*
                args
                ))
  ;;
  (:body-pose
   (&optional pose &rest args)
   (when pose
     (let (target-coords)
       (setq target-coords (send self :copy-worldcoords))
       (if (or (vectorp pose) (listp pose))
           (progn
             (ros::ros-debug "Got r p y float-vector or list as args.")
             (setq pose (instance coordinates :init :rpy (list (elt pose 2) (elt pose 1) (elt pose 0)))))
           (ros::ros-debug "Got coords variable as args."))
       (send target-coords :transform pose)
       (send* self :fullbody-inverse-kinematics
              target-coords (send self :base_lk)
              :rotation-axis t
              :translation-axis t
              :debug-view nil
              :warnp nil
              args)))
     (send self :coords))
  ;;
  (:look-at
   (pos &rest args)
   (let (target-coords)
     (setq target-coords (orient-coords-to-axis
                          (make-coords :pos pos
                                       :rot (copy-object (send self :head-end-coords :worldrot))) ;; orient-coords-to-axis is destructive
                          (normalize-vector (v- pos (send self :head-end-coords :worldpos)))))
     (send* self :fullbody-inverse-kinematics
            target-coords (send self :head-end-coords)
            :rotation-axis :z
            :translation-axis nil
            :debug-view nil
            :warnp nil
            args)
     ))
   )
