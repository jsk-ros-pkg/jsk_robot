(require :gen3 "package://kinovaeus/gen3-lite.l")

(defun gen3-lite () (setq *gen3-lite* (instance gen3_lite_gen3_lite_2f-robot :init)))

(defmethod gen3_lite_gen3_lite_2f-robot
  (:arm (&rest args) (send* self :rarm args)) ;; enable to call send *gen3-lite* :arm :angle-vector
  )
