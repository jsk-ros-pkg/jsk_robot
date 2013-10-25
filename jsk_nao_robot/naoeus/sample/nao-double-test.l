(ros::roseus "nao-double-test")
(load "package://naoeus/euslisp/nao-interface.l")

(setq *nao1* (nao))
(objects (list *nao1*))
(setq *ni1* (instance nao-interface :init :namespace "nao1"))

(setq *nao2* (nao))
(setq *ni2* (instance nao-interface :init :namespace "nao2"))

(unix:sleep 1)
(send *ni1* :servo-on)
(send *ni2* :servo-on)
(dotimes (i 3)
  (send *nao1* :arms :elbow-p :joint-angle (if (evenp i) -20 -90))
  (send *ni1* :angle-vector (send *nao1* :angle-vector) 2000)
  (send *ni1* :wait-interpolation)

  (send *nao2* :arms :elbow-p :joint-angle (if (evenp i) -20 -90))
  (send *ni2* :angle-vector (send *nao2* :angle-vector) 2000)
  (send *ni2* :wait-interpolation)
  )

(send *ni1* :go-pos 1 0 0)
(send *ni2* :go-pos 1 0 0)
