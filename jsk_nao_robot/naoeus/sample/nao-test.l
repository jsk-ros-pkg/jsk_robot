(ros::roseus "nao-test")
(load "package://naoeus/nao-interface.l")

(setq *nao* (nao))
(objects (list *nao*))
(setq *ni* (instance nao-interface :init :namespace "nao"))

(unix:sleep 1)
(send *ni* :servo-on)
(dotimes (i 3)
  (send *nao* :arms :elbow-p :joint-angle (if (evenp i) -20 -90))
  (send *ni* :angle-vector (send *nao* :angle-vector) 2000)
  ;;(send *ni* :wait-interpolation)
  )

(send *ni* :go-pos 1 0 0)
