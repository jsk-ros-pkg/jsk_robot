(require :cobotta "package://cobottaeus/cobotta.l")

(defmethod cobotta-robot
  (:arm (&rest args) (send* self :rarm args))      ;; enable to call send *cobotta* :arm :angle-vector
  )
