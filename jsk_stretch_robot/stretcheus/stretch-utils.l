(require :stretch_description "package://stretcheus/stretch.l")


(defun stretch () (setq *stretch* (instance stretch_description-robot :init)))


(provide :stretch-utils "stretch-utils.l")
