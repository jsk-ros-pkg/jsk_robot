(ros::roseus "nao-test")
(load "package://naoeus/euslisp/nao-interface.l")

(setq *nao* (nao))
(objects (list *nao*))
(setq *ni* (instance nao-interface :init)) ; :namespace "nao"))

(unix:sleep 1)
(send *ni* :servo-on)
(dotimes (i 3)
  (send *nao* :arms :elbow-p :joint-angle (if (evenp i) -20 -90))
  (send *ni* :angle-vector (send *nao* :angle-vector) 5000) ; needs enough time to wait for wait-interpolation
  ;;(send *ni* :wait-interpolation)
  )

(send *ni* :wait-interpolation) ; working...?
(send *ni* :go-pos 0.1 0 0) ; now it walks!