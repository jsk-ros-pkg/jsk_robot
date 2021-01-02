(require :gen3 "package://kinovaeus/gen3.l")

(defun gen3 () (setq *gen3* (instance gen3_robotiq_2f_85-robot :init)))

(defmethod gen3_robotiq_2f_85-robot
  (:arm (&rest args) (send* self :rarm args)) ;; enable to call send *gen3* :arm :angle-vector
  )
