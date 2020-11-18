(load "robot-teleop-joy.l")

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(defmethod hrp2jsknts-interface
  (:start-grasp (arm &key (robot *robot*) (time 3000) (wait nil) &allow-other-keys)
    (send robot :hand arm :index-avoid-grasp-pose)
    (send self :hand-angle-vector (send robot :hand-angle-vector) time)
    (if wait (send self :hand-wait-interpolation)))
  (:stop-grasp (arm &key (robot *robot*) (time 3000) (wait nil) &allow-other-keys)
    (send robot :hand arm :hook-pose)
    (send self :hand-angle-vector (send robot :hand-angle-vector) time)
    (if wait (send self :hand-wait-interpolation)))
  )

(defun hrp2jsknts-teleop-joy-setup ()
  ;; set *robot* and *ri*
  (hrp2jsknts-init)
  (setq *robot* *hrp2jsknts*)
  (when *debug-teleop-joy*
    (unless (boundp '*irtviewer*)
      (make-irtviewer)
      (objects (list *robot*))))
  ;; setup teleop-joy
  (teleop-joy-setup)
  ;; options
  (setq *grasp-loop-enable* t
        *reset-loop-enable* t
        *look-at-loop-enable* t
        *move-head-enable* t)
  )
(warn ";; (hrp2jsknts-teleop-joy-setup)~%")
(warn ";; (run-teleop-joy)~%")
