(load "package://naoeus/nao-utils.l") 
(setq *nao* (NaoH25V50))
(objects (list *nao*)) 
(send *nao* :reset-pose)

;; fullbody-ik
(setq rleg-coords (make-coords :pos (float-vector 120 -170 -250)))
(setq lleg-coords (send *nao* :lleg :end-coords :copy-worldcoords))
(send *nao* :fullbody-inverse-kinematics
      (list rleg-coords
	    lleg-coords)
      :move-target
      (list (send *nao* :rleg :end-coords)
	    (send *nao* :lleg :end-coords))
      :link-list
      (list (send *nao* :link-list (send *nao* :rleg :end-coords :parent))
	    (send *nao* :link-list (send *nao* :lleg :end-coords :parent)))
      :translation-axis (list t t)
      :rotation-axis (list nil t)
      :target-centroid-pos (send *nao* :lleg :end-coords :worldpos)
      :cog-translation-axis :z
      :thre (list 30 30)
      :centroid-thre 5
      )

(objects (list *nao* lleg-coords rleg-coords))

(format t "crotch-y angles must be the same: ~A~%"(send *nao* :legs :crotch-y :joint-angle))
