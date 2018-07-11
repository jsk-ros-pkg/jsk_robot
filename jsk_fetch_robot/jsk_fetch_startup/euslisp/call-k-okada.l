;; go-dock.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "call_k_okada")
(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(defun call-k-okada ()
  ;; call k-okada
  (send *ri* :speak "call k-okada")
  (go-to-spot "/eng2/7f/room73A2-front")
  (send *ri* :speak "Hello k-okada" :wait t)
  (unix:sleep 1)
  (send *ri* :speak "Hello k-okada" :wait t)

  ;; auto docking
  (auto-dock))

(call-k-okada)
