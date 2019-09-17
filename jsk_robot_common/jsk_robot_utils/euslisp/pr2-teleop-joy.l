(load "robot-teleop-joy.l")

(defun pr2-teleop-joy-setup ()
  ;; set *robot* and *ri*
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  (setq *robot* *pr2*)
  (when *debug-teleop-joy*
    (unless (boundp '*irtviewer*)
      (make-irtviewer)
      (objects (list *robot*))))
  ;; setup teleop-joy
  (teleop-joy-setup)
  ;; options
  (setq *grasp-loop-enable* nil
        *reset-loop-enable* nil
        *look-at-loop-enable* nil
        *move-head-enable* nil)
  )
(warn ";; (pr2-teleop-joy-setup)~%")
(warn ";; (run-teleop-joy)~%")
