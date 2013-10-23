(load "package://naoeus/euslisp/nao-interface.l")

(defun mirror (&key (ni-master *ni-master*) (ni-slave *ni-slave*))
  (send ni-slave :angle-vector (send ni-master :state :angle-vector) 100)
  (send ni-slave :wait-interpolation)
  )

(defun mirror-to-model (&key (ni-master *ni-master*) (ni-slave *nao*))
  (do-until-key
   (send ni-slave :angle-vector (send ni-master :state :angle-vector))
   ;;(print (send ni-master :state :angle-vector))
   (send *irtviewer* :draw-objects)
   (x::window-main-one)
   )
  )

(defun master-slave (&key (ni-master *ni-master*) (ni-slave *ni-slave*))
  (send ni-slave :servo-on)
  (do-until-key
   (send ni-slave :angle-vector (send ni-master :state :angle-vector) 100)
   ;;(send ni-slave :angle-vector (send ni-master :state :angle-vector))
   (send ni-slave :wait-interpolation)
   ;;(send *irtviewer* :draw-objects)
   )
  (send ni-slave :servo-off)
  )

(setq *ni-master* (instance nao-interface :init :namespace "nao_master"))
(setq *ni-slave* (instance nao-interface :init :namespace "nao_slave"))
(setq *nao* (nao))
(objects (list *nao*))