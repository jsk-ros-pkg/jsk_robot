(load "robot-teleop.l")

(defun pr2-teleop-setup ()
  ;; set *robot* and *ri*
  (load "package://pr2eus/pr2-interface.l")
  (pr2-init)
  (setq *robot* *pr2*)
  (when *debug-teleop*
    (unless (boundp '*irtviewer*)
      (make-irtviewer)
      (objects (list *robot*))))
  ;; setup teleop
  (teleop-setup)
  ;; options
  (setq *grasp-loop-enable* nil
        *reset-loop-enable* nil
        *look-at-loop-enable* nil
        *move-head-enable* nil)
  )
(warn ";; (pr2-teleop-setup)~%")
(warn ";; (run-teleop)~%")

