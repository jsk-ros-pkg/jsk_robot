(require :panda-interface "package://panda_eus/euslisp/panda-interface.l")
(require :fr3-utils "package://panda_eus/euslisp/fr3-utils.l")

(defclass fr3-robot-interface
  :super panda-robot-interface
  :slots ())

(defmethod fr3-robot-interface
  (:init
    (&rest args)
    (send-super* :init :robot fr3-robot
                 args))
  )

(defun fr3-init ()
  (setq *ri* (instance fr3-robot-interface :init))
  (setq *robot* (fr3)))

(provide :fr3-interface)
