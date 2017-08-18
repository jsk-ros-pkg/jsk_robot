;; pr2-logging-interface.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :action-logging-utils "package://jsk_robot_startup/lifelog/action-logging-utils.l")

;; pr2-interface
(defclass pr2-logging-interface
  :super pr2-interface)

(declare-logging-method pr2-logging-interface :wait-interpolation :prefunc #'(lambda (x) (mapcar #'null x)))
(declare-logging-method pr2-logging-interface :move-to)
(declare-logging-method pr2-logging-interface :go-pos)
(declare-logging-method pr2-logging-interface :go-pos-unsafe)
(declare-logging-method pr2-logging-interface :move-gripper)
;; (declare-logging-method pr2-logging-interface :angle-vector) ;; :angle-vector calls :angle-vector-sequence inside
(declare-logging-method pr2-logging-interface :angle-vector-sequence)

(unless (fboundp 'pr2-init-org)
  (setf (symbol-function 'pr2-init-org) (symbol-function 'pr2-init)))

(defun pr2-init (&rest args)
  (unless (boundp '*ri*)
    (setq *ri* (instance pr2-logging-interface :init)))
  (apply #'pr2-init-org args))


(provide :pr2-logging-interface) ;; end of pr2-logging-interface.l
