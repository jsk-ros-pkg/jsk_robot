(load "robot-teleop-joy.l")

(defun aero-teleop-joy-setup ()
  ;; set *robot* and *ri*
  (load "package://aeroeus/aero-interface.l")
  (aero-init)
  (setq *robot* *aero*)
  (when *debug-teleop-joy*
    (unless (boundp '*irtviewer*)
      (make-irtviewer)
      (objects (list *robot*))))
  ;; setup teleop-joy
  (setq *joy-type* :ps4)
  (teleop-joy-setup)
  ;; options
  (setq *grasp-loop-enable* nil
        *reset-loop-enable* nil
        *look-at-loop-enable* nil
        *move-head-enable* nil)
  )
(warn ";; (aero-teleop-joy-setup)~%")
(warn ";; (run-teleop-joy :reset-pose :reset-pose)~%")
