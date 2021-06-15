(load "package://cobottaeus/cobotta-interface.l")

(defun send-ri-angle-vector (av tm &optional duration-factor)
  (let (rael-tm ratio)
    (format t "~A -> ~A, tm: ~A~%" (send *ri* :state :potentio-vector) av tm)
    (setq real-tm (bench2
                   (if duration-factor
                       (send *ri* :angle-vector av tm duration-factor)
                     (send *ri* :angle-vector av tm))))
    (setq ratio (/ (/ tm 1000.0) real-tm))
    (format t "real-tm: ~A duration-factor: ~A~%" real-tm ratio)
    ratio))

(defun check-duratoin-factor ()
  (let ((av #f(-20.0 20.0 80.0 0.0 20.0 0.0))
        (ratio-list) (duration-list))
    (dotimes (i (length av))
      (setq av-diff (instantiate float-vector (length av)))
      (setf (elt av-diff i) 20)
      (push (send-ri-angle-vector (v+ av av-diff) 2000) duration-list)
      (push (send-ri-angle-vector av 2000 1.0) ratio-list)
      (push (send-ri-angle-vector (v+ av av-diff) 4000) duration-list)
      (push (send-ri-angle-vector av 4000 1.0) ratio-list))
    (format t "mean duration-factor is ~A, please update default :duratoin-factor of :move-robot method in cobotta-interface.l to ~A~%" (vmean duration-list) (vmean ratio-list))
    ))

(check-duratoin-factor)

